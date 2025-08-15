"""Tripo3D image-to-3D generation implementation."""

import asyncio
import base64
import os
import time
from pathlib import Path
from typing import Optional, Dict, Any
import httpx

from .base import Image3DGenerator, GenerationResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError, ConfigurationError
from ..utils.validation import validate_image_file


class TripoGenerator(Image3DGenerator):
    """Tripo3D image-to-3D generator."""
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize Tripo generator.
        
        Args:
            name: Name of the generator.
            config: Configuration dictionary with Tripo API settings.
        """
        super().__init__(name, config)
        
        # API configuration - only official Tripo3D API is supported now
        self.api_key = (
            os.getenv("TRIPO_API_KEY") or 
            os.getenv("FAL_KEY") or 
            self.get_config_value("tripo.api_key", None)
        )
        self.base_url = self.get_config_value("tripo.base_url", "https://api.tripo3d.ai/v2/openapi/task")
        
        # Generation parameters
        self.texture_quality = self.get_config_value("tripo.texture_quality", "standard")  # "no", "standard", "HD"
        self.face_limit = self.get_config_value("tripo.face_limit", 10000)
        self.enable_pbr = self.get_config_value("tripo.enable_pbr", False)
        self.enable_quad = self.get_config_value("tripo.enable_quad", False)
        self.auto_size = self.get_config_value("tripo.auto_size", True)
        self.style = self.get_config_value("tripo.style", None)  # e.g., "object:clay", "person:person2cartoon"
        self.seed = self.get_config_value("tripo.seed", None)
        
        # Polling configuration
        self.poll_interval = self.get_config_value("tripo.poll_interval", 5)  # seconds
        self.max_wait_time = self.get_config_value("tripo.max_wait_time", 600)  # 10 minutes
        
        if not self.api_key:
            raise ConfigurationError(
                "Tripo API key is required. Set 'TRIPO_API_KEY' environment variable "
                "or 'tripo.api_key' in configuration. Get your key at: https://platform.tripo3d.ai/api-keys"
            )
        
        if not self.api_key.startswith("tsk_"):
            raise ConfigurationError(
                "Only official Tripo3D API keys (starting with 'tsk_') are supported. "
                "Get your key at: https://platform.tripo3d.ai/api-keys"
            )
    
    async def generate_3d(self, hero: Hero) -> GenerationResult:
        """Generate 3D mesh using Tripo3D API.
        
        Args:
            hero: Hero with input image.
            
        Returns:
            Generation result with mesh path and metadata.
            
        Raises:
            ProcessingError: If generation fails.
        """
        start_time = time.time()
        
        # Get input image
        input_image = hero.assets.get_asset(AssetType.INPUT_IMAGE)
        self.logger.info(f"Starting Tripo generation for {hero.name} with image {input_image}")
        
        # Upload image and get file_token
        file_token, file_type = await self._upload_image(input_image)
        
        # Create generation task
        task_result = await self._create_and_wait_for_task(file_token, file_type)
        
        # Download the generated mesh
        mesh_path = await self._download_mesh(task_result, hero)
        
        generation_time = time.time() - start_time
        
        # Calculate quality score based on task result
        quality_score = self._calculate_quality_score(task_result)
        
        # Prepare metadata
        metadata = {
            "texture_quality": self.texture_quality,
            "face_limit": self.face_limit,
            "pbr_enabled": self.enable_pbr,
            "quad_mesh": self.enable_quad,
            "auto_size": self.auto_size,
            "style": self.style,
            "seed": self.seed,
            "tripo_model_url": task_result.get("model_mesh"),
            "tripo_preview": task_result.get("rendered_image")
        }
        
        return GenerationResult(
            mesh_path=mesh_path,
            quality_score=quality_score,
            generation_time=generation_time,
            metadata=metadata
        )
    
    async def _upload_image(self, image_path: Path) -> str:
        """Upload image to Tripo API and get file_token.
        
        Args:
            image_path: Path to input image.
            
        Returns:
            File token for the uploaded image.
        """
        try:
            # Determine content type from file extension
            suffix = image_path.suffix.lower()
            if suffix == '.png':
                content_type = 'image/png'
                file_type = 'png'
            elif suffix in ['.jpg', '.jpeg']:
                content_type = 'image/jpeg'
                file_type = 'jpg'
            else:
                raise ProcessingError(f"Unsupported image format: {suffix}")
            
            upload_url = "https://api.tripo3d.ai/v2/openapi/upload"
            headers = {
                "Authorization": f"Bearer {self.api_key}"
            }
            
            async with httpx.AsyncClient(timeout=60.0, trust_env=False) as client:
                with open(image_path, 'rb') as f:
                    files = {"file": (image_path.name, f, content_type)}
                    
                    self.logger.info(f"Uploading image to Tripo API: {image_path.name}")
                    response = await client.post(upload_url, headers=headers, files=files)
                    
                    if response.status_code != 200:
                        raise ProcessingError(f"Failed to upload image: {response.status_code} - {response.text}")
                    
                    result = response.json()
                    
                    if "data" in result and ("file_token" in result["data"] or "image_token" in result["data"]):
                        file_token = result["data"].get("file_token") or result["data"].get("image_token")
                        self.logger.info(f"Successfully uploaded image, token: {file_token}")
                        return file_token, file_type
                    else:
                        raise ProcessingError(f"Unexpected upload response: {result}")
                        
        except Exception as e:
            raise ProcessingError(f"Failed to upload image to Tripo API: {e}")
    
    async def _create_and_wait_for_task(self, file_token: str, file_type: str) -> Dict[str, Any]:
        """Create image-to-3D task and wait for completion.
        
        Args:
            file_token: File token from uploaded image.
            file_type: File type (jpg, png).
            
        Returns:
            Completed task result.
        """
        # Official Tripo3D API format - based on the provided example
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        # Build payload for Tripo3D API - try both formats
        payload = {
            "type": "image_to_model",
            "file": {
                "type": file_type,
                "file_token": file_token
            }
        }
        
        # Add optional parameters
        if self.face_limit != 10000:
            payload["face_limit"] = self.face_limit
        
        if self.texture_quality != "standard":
            payload["texture_quality"] = self.texture_quality
        
        if self.enable_pbr:
            payload["enable_pbr"] = self.enable_pbr
            
        if self.enable_quad:
            payload["enable_quad"] = self.enable_quad
            
        if self.style:
            payload["style"] = self.style
            
        if self.seed is not None:
            payload["seed"] = self.seed
        
        self.logger.info(f"Using official Tripo3D API payload: {payload}")
        
        try:
            async with httpx.AsyncClient(timeout=60.0, trust_env=False) as client:
                self.logger.info(f"Creating Tripo task with file_token...")
                
                response = await client.post(
                    self.base_url,
                    headers=headers,
                    json=payload
                )
                
                if response.status_code != 200:
                    # Enhanced error handling with specific messages
                    if response.status_code == 400:
                        try:
                            error_data = response.json()
                            error_code = error_data.get("code")
                            error_msg = error_data.get("message", "Unknown error")
                            
                            if error_code == 1004:  # Parameter invalid
                                error_msg = (
                                    f"Tripo API parameter error: {error_msg}. "
                                    "This may indicate that your account doesn't have access to image_to_model tasks, "
                                    "or additional parameters are required. Please check your account status at "
                                    "https://platform.tripo3d.ai and ensure you have the necessary permissions."
                                )
                            elif error_code == 1003:  # Malformed request
                                error_msg = (
                                    f"Tripo API request format error: {error_msg}. "
                                    "The request body format may need to be adjusted."
                                )
                            elif error_code == 2017:  # Version error
                                error_msg = (
                                    f"Tripo API version error: {error_msg}. "
                                    "The model version parameter may be incorrect."
                                )
                            elif error_code == 2002:  # Task type not supported
                                error_msg = (
                                    f"Tripo API task type error: {error_msg}. "
                                    "The image_to_model task type may not be supported on your account."
                                )
                            else:
                                error_msg = f"Tripo API error {error_code}: {error_msg}"
                                
                        except:
                            error_msg = f"Tripo API error 400: {response.text}"
                    elif response.status_code == 403:
                        error_msg = (
                            f"Tripo API access denied: {response.text}. "
                            "This usually means insufficient credits or account permissions. "
                            "Please check your account balance and subscription at https://platform.tripo3d.ai"
                        )
                    elif response.status_code == 401:
                        error_msg = (
                            f"Tripo API authentication failed: {response.text}. "
                            "Please check your API key is correct and set in FAL_KEY or TRIPO_API_KEY environment variable."
                        )
                    else:
                        error_msg = f"Tripo API error {response.status_code}: {response.text}"
                    
                    raise ProcessingError(error_msg)
                
                result = response.json()
                self.logger.debug(f"API response: {result}")
                
                # Official Tripo3D API response handling
                if "data" in result and "task_id" in result["data"]:
                    task_id = result["data"]["task_id"]
                    self.logger.info(f"Created Tripo task: {task_id}")
                    return await self._poll_task(task_id)
                elif "task_id" in result:
                    task_id = result["task_id"]
                    self.logger.info(f"Created Tripo task: {task_id}")
                    return await self._poll_task(task_id)
                else:
                    raise ProcessingError(f"Unexpected Tripo API response: {result}")
                
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to create Tripo task: {e}")
    
    async def _poll_task(self, task_id: str) -> Dict[str, Any]:
        """Poll task status until completion.
        
        Args:
            task_id: Task ID to poll.
            
        Returns:
            Completed task result.
        """
        return await self._poll_official_task(task_id)
    
    async def _poll_official_task(self, task_id: str) -> Dict[str, Any]:
        """Poll official Tripo3D API task status until completion.
        
        Args:
            task_id: Task ID to poll.
            
        Returns:
            Completed task result.
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        poll_url = f"https://api.tripo3d.ai/v2/openapi/task/{task_id}"
        start_time = time.time()
        
        try:
            async with httpx.AsyncClient(timeout=30.0, trust_env=False) as client:
                while True:
                    # Check if we've exceeded max wait time
                    if time.time() - start_time > self.max_wait_time:
                        raise ProcessingError(f"Tripo task {task_id} timed out after {self.max_wait_time}s")
                    
                    # Get task status
                    response = await client.get(poll_url, headers=headers)
                    
                    if response.status_code != 200:
                        error_msg = f"Failed to get task status {response.status_code}: {response.text}"
                        raise ProcessingError(error_msg)
                    
                    result = response.json()
                    
                    if "data" in result:
                        data = result["data"]
                        status = data.get("status")
                        progress = data.get("progress", 0)
                        
                        self.logger.info(f"Tripo task {task_id} status: {status} ({progress}%)")
                        
                        if status == "success":
                            return data
                        elif status == "failed":
                            error = data.get("error", "Unknown error")
                            raise ProcessingError(f"Tripo task failed: {error}")
                        elif status in ["queued", "running"]:
                            # Wait before next poll
                            await asyncio.sleep(self.poll_interval)
                        else:
                            self.logger.warning(f"Unknown Tripo task status: {status}")
                            await asyncio.sleep(self.poll_interval)
                    else:
                        raise ProcessingError(f"Unexpected polling response: {result}")
                        
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to poll Tripo task: {e}")
    
    async def _download_mesh(self, task_result: Dict[str, Any], hero: Hero) -> Path:
        """Download generated mesh file.
        
        Args:
            task_result: Completed task result from Tripo.
            hero: Hero for output path.
            
        Returns:
            Path to downloaded mesh file.
        """
        # Official Tripo3D API response format - check multiple possible locations
        model_url = None
        
        # Try result field first
        if "result" in task_result:
            result = task_result["result"]
            if isinstance(result, dict):
                # Look for various possible field names and ensure we get a string URL
                for field_name in ["model_mesh", "model", "pbr_model", "rendered_image"]:
                    field_value = result.get(field_name)
                    if field_value and isinstance(field_value, str):
                        model_url = field_value
                        break
            elif isinstance(result, str):
                model_url = result
        
        # Try output field as fallback
        if not model_url and "output" in task_result:
            output = task_result["output"]
            if isinstance(output, dict):
                for field_name in ["model_mesh", "model", "pbr_model"]:
                    field_value = output.get(field_name)
                    if field_value and isinstance(field_value, str):
                        model_url = field_value
                        break
            elif isinstance(output, str):
                model_url = output
                
        # Try direct fields - ensure we get string values
        if not model_url:
            for field_name in ["model_mesh", "model", "pbr_model"]:
                field_value = task_result.get(field_name)
                if field_value and isinstance(field_value, str):
                    model_url = field_value
                    break
        
        if not model_url:
            # Debug logging to understand the response structure
            self.logger.error(f"Could not find model URL in response")
            self.logger.error(f"Available keys: {list(task_result.keys())}")
            if "result" in task_result:
                self.logger.error(f"Result content: {task_result['result']}")
            if "output" in task_result:
                self.logger.error(f"Output content: {task_result['output']}")
            
            raise ProcessingError(f"No model mesh URL in Tripo response. Available keys: {list(task_result.keys())}")
        
        # Determine file extension from URL or default to GLB
        if model_url.endswith('.obj'):
            file_extension = ".obj"
        elif model_url.endswith('.fbx'):
            file_extension = ".fbx"
        elif model_url.endswith('.glb'):
            file_extension = ".glb"
        else:
            file_extension = ".glb"  # Default format
        
        output_path = hero.hero_dir / f"generated_mesh{file_extension}"
        
        try:
            async with httpx.AsyncClient(timeout=300.0, trust_env=False) as client:  # 5 min timeout for download
                response = await client.get(model_url)
                
                if response.status_code != 200:
                    raise ProcessingError(f"Failed to download mesh: {response.status_code}")
                
                with open(output_path, 'wb') as f:
                    f.write(response.content)
                
                self.logger.info(f"Downloaded mesh to {output_path} (size: {output_path.stat().st_size} bytes)")
                return output_path
                
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to download mesh from Tripo: {e}")
    
    def _calculate_quality_score(self, task_result: Dict[str, Any]) -> float:
        """Calculate quality score based on task result.
        
        Args:
            task_result: Completed task result.
            
        Returns:
            Quality score between 0.0 and 1.0.
        """
        # Base score
        score = 0.8
        
        # Adjust based on texture quality
        if self.texture_quality == "HD":
            score += 0.15
        elif self.texture_quality == "standard":
            score += 0.1
        
        # Adjust if PBR is enabled
        if self.enable_pbr:
            score += 0.05
        
        return min(score, 1.0)
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate inputs for Tripo generation.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
        """
        # Call parent validation
        super().validate_inputs(hero)
        
        # Additional Tripo-specific validation
        input_image = hero.assets.get_asset(AssetType.INPUT_IMAGE)
        
        # Check file size (reasonable limit)
        file_size = input_image.stat().st_size
        max_size = 20 * 1024 * 1024  # 20MB limit
        if file_size > max_size:
            raise ValidationError(f"Image file too large for Tripo API: {file_size} bytes > {max_size} bytes")
        
        # Check image format
        suffix = input_image.suffix.lower()
        supported_formats = ['.jpg', '.jpeg', '.png']
        if suffix not in supported_formats:
            raise ValidationError(f"Unsupported image format for Tripo: {suffix}. Supported: {supported_formats}")
        
        return True
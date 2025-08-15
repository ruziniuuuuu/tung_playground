"""Meshy AI image-to-3D generation implementation."""

import asyncio
import base64
import json
import os
import time
from pathlib import Path
from typing import Optional, Dict, Any
from urllib.parse import urlparse
import httpx

from .base import Image3DGenerator, GenerationResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError, ConfigurationError
from ..utils.validation import validate_image_file


class MeshyGenerator(Image3DGenerator):
    """Meshy AI image-to-3D generator."""
    
    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize Meshy generator.
        
        Args:
            name: Name of the generator.
            config: Configuration dictionary with Meshy API settings.
        """
        super().__init__(name, config)
        
        # API configuration - try environment variable first, then config
        self.api_key = (
            os.getenv("MESHY_API_KEY") or 
            self.get_config_value("meshy.api_key", None)
        )
        self.base_url = self.get_config_value("meshy.base_url", "https://api.meshy.ai")
        self.model = self.get_config_value("meshy.model", "meshy-5")
        self.topology = self.get_config_value("meshy.topology", "triangle")
        self.target_polycount = self.get_config_value("meshy.target_polycount", 10000)
        self.should_texture = self.get_config_value("meshy.should_texture", True)
        self.enable_pbr = self.get_config_value("meshy.enable_pbr", False)
        self.texture_prompt = self.get_config_value("meshy.texture_prompt", "")
        self.content_safety = self.get_config_value("meshy.content_safety", True)
        
        # Polling configuration
        self.poll_interval = self.get_config_value("meshy.poll_interval", 10)  # seconds
        self.max_wait_time = self.get_config_value("meshy.max_wait_time", 1800)  # 30 minutes
        
        if not self.api_key:
            raise ConfigurationError(
                "Meshy API key is required. Set 'MESHY_API_KEY' environment variable "
                "or 'meshy.api_key' in configuration."
            )
    
    async def generate_3d(self, hero: Hero) -> GenerationResult:
        """Generate 3D mesh using Meshy AI API.
        
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
        self.logger.info(f"Starting Meshy generation for {hero.name} with image {input_image}")
        
        # Convert image to base64 data URI or use public URL
        image_url = await self._prepare_image(input_image)
        
        # Create generation task
        task_id = await self._create_task(image_url)
        self.logger.info(f"Created Meshy task {task_id}")
        
        # Poll for completion
        task_result = await self._poll_task(task_id)
        
        # Download the generated mesh
        mesh_path = await self._download_mesh(task_result, hero)
        
        generation_time = time.time() - start_time
        
        # Calculate quality score based on task result
        quality_score = self._calculate_quality_score(task_result)
        
        # Prepare metadata
        metadata = {
            "task_id": task_id,
            "model": self.model,
            "topology": self.topology,
            "polycount": task_result.get("polygon_count", self.target_polycount),
            "textured": self.should_texture,
            "pbr_enabled": self.enable_pbr,
            "meshy_thumbnail": task_result.get("thumbnail_url"),
            "meshy_model_urls": task_result.get("model_urls", {})
        }
        
        return GenerationResult(
            mesh_path=mesh_path,
            quality_score=quality_score,
            generation_time=generation_time,
            metadata=metadata
        )
    
    async def _prepare_image(self, image_path: Path) -> str:
        """Prepare image for Meshy API (convert to data URI).
        
        Args:
            image_path: Path to input image.
            
        Returns:
            Data URI string for the image.
        """
        try:
            # Read image file
            with open(image_path, 'rb') as f:
                image_data = f.read()
            
            # Determine MIME type
            suffix = image_path.suffix.lower()
            if suffix == '.png':
                mime_type = 'image/png'
            elif suffix in ['.jpg', '.jpeg']:
                mime_type = 'image/jpeg'
            else:
                raise ProcessingError(f"Unsupported image format: {suffix}")
            
            # Encode as base64 data URI
            base64_data = base64.b64encode(image_data).decode('utf-8')
            data_uri = f"data:{mime_type};base64,{base64_data}"
            
            self.logger.debug(f"Converted image to data URI (size: {len(data_uri)} chars)")
            return data_uri
            
        except Exception as e:
            raise ProcessingError(f"Failed to prepare image for Meshy API: {e}")
    
    async def _create_task(self, image_url: str) -> str:
        """Create image-to-3D task on Meshy API.
        
        Args:
            image_url: Image data URI or public URL.
            
        Returns:
            Task ID for the generation.
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "image_url": image_url,
            "ai_model": self.model,
            "topology": self.topology,
            "target_polycount": self.target_polycount,
            "should_texture": self.should_texture,
            "enable_pbr": self.enable_pbr,
            "content_safety": self.content_safety
        }
        
        if self.texture_prompt:
            payload["texture_prompt"] = self.texture_prompt
        
        try:
            async with httpx.AsyncClient(timeout=60.0) as client:
                response = await client.post(
                    f"{self.base_url}/openapi/v1/image-to-3d",
                    headers=headers,
                    json=payload
                )
                
                if response.status_code != 200:
                    error_msg = f"Meshy API error {response.status_code}: {response.text}"
                    raise ProcessingError(error_msg)
                
                result = response.json()
                task_id = result.get("result")
                
                if not task_id:
                    raise ProcessingError(f"No task ID returned from Meshy API: {result}")
                
                return task_id
                
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to create Meshy task: {e}")
    
    async def _poll_task(self, task_id: str) -> Dict[str, Any]:
        """Poll task status until completion.
        
        Args:
            task_id: Task ID to poll.
            
        Returns:
            Completed task result.
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}"
        }
        
        start_time = time.time()
        
        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                while True:
                    # Check if we've exceeded max wait time
                    if time.time() - start_time > self.max_wait_time:
                        raise ProcessingError(f"Meshy task {task_id} timed out after {self.max_wait_time}s")
                    
                    # Get task status
                    response = await client.get(
                        f"{self.base_url}/openapi/v1/image-to-3d/{task_id}",
                        headers=headers
                    )
                    
                    if response.status_code != 200:
                        error_msg = f"Failed to get task status {response.status_code}: {response.text}"
                        raise ProcessingError(error_msg)
                    
                    result = response.json()
                    status = result.get("status")
                    progress = result.get("progress", 0)
                    
                    self.logger.info(f"Meshy task {task_id} status: {status} ({progress}%)")
                    
                    if status == "SUCCEEDED":
                        return result
                    elif status == "FAILED":
                        error = result.get("error", "Unknown error")
                        raise ProcessingError(f"Meshy task failed: {error}")
                    elif status in ["PENDING", "IN_PROGRESS"]:
                        # Wait before next poll
                        await asyncio.sleep(self.poll_interval)
                    else:
                        self.logger.warning(f"Unknown Meshy task status: {status}")
                        await asyncio.sleep(self.poll_interval)
                        
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to poll Meshy task: {e}")
    
    async def _download_mesh(self, task_result: Dict[str, Any], hero: Hero) -> Path:
        """Download generated mesh file.
        
        Args:
            task_result: Completed task result from Meshy.
            hero: Hero for output path.
            
        Returns:
            Path to downloaded mesh file.
        """
        model_urls = task_result.get("model_urls", {})
        
        # Try to get the best available format
        download_url = None
        file_extension = None
        
        # Prefer formats in order: OBJ, GLB, FBX
        for format_name, format_url in model_urls.items():
            if format_name.upper() == "OBJ":
                download_url = format_url
                file_extension = ".obj"
                break
            elif format_name.upper() == "GLB" and not download_url:
                download_url = format_url
                file_extension = ".glb"
            elif format_name.upper() == "FBX" and not download_url:
                download_url = format_url
                file_extension = ".fbx"
        
        if not download_url:
            available_formats = list(model_urls.keys())
            raise ProcessingError(f"No suitable mesh format available. Available: {available_formats}")
        
        # Download the mesh
        output_path = hero.directory / f"generated_mesh{file_extension}"
        
        try:
            async with httpx.AsyncClient(timeout=300.0) as client:  # 5 min timeout for download
                response = await client.get(download_url)
                
                if response.status_code != 200:
                    raise ProcessingError(f"Failed to download mesh: {response.status_code}")
                
                with open(output_path, 'wb') as f:
                    f.write(response.content)
                
                self.logger.info(f"Downloaded mesh to {output_path} (size: {output_path.stat().st_size} bytes)")
                return output_path
                
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to download mesh from Meshy: {e}")
    
    def _calculate_quality_score(self, task_result: Dict[str, Any]) -> float:
        """Calculate quality score based on task result.
        
        Args:
            task_result: Completed task result.
            
        Returns:
            Quality score between 0.0 and 1.0.
        """
        # Base score
        score = 0.8
        
        # Adjust based on polygon count (closer to target is better)
        polygon_count = task_result.get("polygon_count", 0)
        if polygon_count > 0:
            target_ratio = min(polygon_count / self.target_polycount, self.target_polycount / polygon_count)
            score += 0.1 * target_ratio
        
        # Adjust if textured
        if self.should_texture and task_result.get("textured", False):
            score += 0.1
        
        return min(score, 1.0)
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate inputs for Meshy generation.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
        """
        # Call parent validation
        super().validate_inputs(hero)
        
        # Additional Meshy-specific validation
        input_image = hero.assets.get_asset(AssetType.INPUT_IMAGE)
        
        # Check file size (Meshy has limits)
        file_size = input_image.stat().st_size
        max_size = 10 * 1024 * 1024  # 10MB limit
        if file_size > max_size:
            raise ValidationError(f"Image file too large for Meshy API: {file_size} bytes > {max_size} bytes")
        
        # Check image format
        suffix = input_image.suffix.lower()
        supported_formats = ['.jpg', '.jpeg', '.png']
        if suffix not in supported_formats:
            raise ValidationError(f"Unsupported image format for Meshy: {suffix}. Supported: {supported_formats}")
        
        return True
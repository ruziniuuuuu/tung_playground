"""Tripo3D mesh segmentation implementation for part decomposition."""

import asyncio
import time
import os
from pathlib import Path
from typing import Optional, Dict, Any, List
import httpx

from .base import PartDecomposer, DecompositionResult
from ..core.hero import Hero, AssetType
from ..core.exceptions import ProcessingError, ValidationError, ConfigurationError
from ..utils.validation import validate_mesh_file


class TripoDecomposer(PartDecomposer):
    """Tripo3D mesh segmentation for automatic part decomposition."""
    
    def __init__(self, name: str = "tripo_decomposer", config: Optional[Dict[str, Any]] = None):
        """Initialize Tripo decomposer.
        
        Args:
            name: Name of the decomposer.
            config: Configuration dictionary with Tripo API settings.
        """
        super().__init__(name, config)
        
        # API configuration
        self.api_key = (
            os.getenv("TRIPO_API_KEY") or 
            os.getenv("FAL_KEY") or 
            self.get_config_value("tripo.api_key", None)
        )
        self.base_url = self.get_config_value("tripo.base_url", "https://api.tripo3d.ai/v2/openapi/task")
        
        # Segmentation parameters
        self.segmentation_method = self.get_config_value("tripo.segmentation_method", "semantic")  # "semantic", "geometric"
        self.num_segments = self.get_config_value("tripo.num_segments", None)  # Auto if None
        self.smoothness = self.get_config_value("tripo.smoothness", 0.5)  # 0.0-1.0
        self.detail_level = self.get_config_value("tripo.detail_level", "medium")  # "low", "medium", "high"
        
        # Output format
        self.output_format = self.get_config_value("tripo.output_format", "obj")  # obj, glb, ply
        self.generate_labels = self.get_config_value("tripo.generate_labels", True)
        
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
    
    async def decompose_parts(self, hero: Hero) -> DecompositionResult:
        """Decompose 3D mesh into semantic parts using Tripo3D segmentation.
        
        Args:
            hero: Hero with 3D mesh.
            
        Returns:
            Decomposition result with part paths and metadata.
        """
        start_time = time.time()
        
        # Get input mesh
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        self.logger.info(f"Starting Tripo mesh segmentation for {hero.name} with mesh {mesh_path}")
        
        # Upload mesh and get file_token
        file_token = await self._upload_mesh(mesh_path)
        
        # Create segmentation task
        task_result = await self._create_and_wait_for_segmentation_task(file_token)
        
        # Download the segmented parts
        parts_paths, part_names = await self._download_segmented_parts(task_result, hero)
        
        decomposition_time = time.time() - start_time
        
        # Prepare metadata
        metadata = {
            "segmentation_method": self.segmentation_method,
            "num_segments": len(parts_paths),
            "smoothness": self.smoothness,
            "detail_level": self.detail_level,
            "output_format": self.output_format,
            "api_version": "tripo_v2",
            "task_id": task_result.get("task_id"),
            "labels_generated": self.generate_labels
        }
        
        return DecompositionResult(
            parts_paths=parts_paths,
            part_names=part_names,
            decomposition_time=decomposition_time,
            metadata=metadata
        )
    
    async def _upload_mesh(self, mesh_path: Path) -> str:
        """Upload mesh to Tripo API and get file_token.
        
        Args:
            mesh_path: Path to input mesh.
            
        Returns:
            File token for the uploaded mesh.
        """
        try:
            # Determine content type from file extension
            suffix = mesh_path.suffix.lower()
            content_type_map = {
                '.obj': 'model/obj',
                '.glb': 'model/gltf-binary',
                '.gltf': 'model/gltf+json',
                '.ply': 'application/octet-stream',
                '.stl': 'model/stl'
            }
            
            if suffix not in content_type_map:
                raise ProcessingError(f"Unsupported mesh format for Tripo segmentation: {suffix}")
            
            content_type = content_type_map[suffix]
            
            upload_url = "https://api.tripo3d.ai/v2/openapi/upload"
            headers = {
                "Authorization": f"Bearer {self.api_key}"
            }
            
            async with httpx.AsyncClient(timeout=120.0, trust_env=False) as client:
                with open(mesh_path, 'rb') as f:
                    files = {"file": (mesh_path.name, f, content_type)}
                    
                    self.logger.info(f"Uploading mesh to Tripo API: {mesh_path.name}")
                    self.logger.info(f"Uploading to {upload_url} with headers: {headers}")
                    response = await client.post(upload_url, headers=headers, files=files)
                    
                    self.logger.info(f"Upload response status: {response.status_code}")
                    if response.status_code != 200:
                        self.logger.error(f"Upload failed with status {response.status_code}: {response.text}")
                        raise ProcessingError(f"Failed to upload mesh: {response.status_code} - {response.text}")
                    
                    result = response.json()
                    self.logger.info(f"Upload response: {result}")
                    
                    if "data" in result and ("file_token" in result["data"] or "mesh_token" in result["data"]):
                        file_token = result["data"].get("file_token") or result["data"].get("mesh_token")
                        self.logger.info(f"Successfully uploaded mesh, token: {file_token}")
                        return file_token
                    else:
                        raise ProcessingError(f"Unexpected upload response: {result}")
                        
        except Exception as e:
            raise ProcessingError(f"Failed to upload mesh to Tripo API: {e}")
    
    async def _create_and_wait_for_segmentation_task(self, file_token: str) -> Dict[str, Any]:
        """Create mesh segmentation task and wait for completion.
        
        Args:
            file_token: File token from uploaded mesh.
            
        Returns:
            Completed task result.
        """
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        
        # Build payload for mesh segmentation
        # Note: This is based on typical Tripo API patterns - may need adjustment based on actual API
        payload = {
            "type": "mesh_segmentation",  # Assuming this is the task type
            "file": {
                "file_token": file_token
            },
            "options": {
                "method": self.segmentation_method,
                "smoothness": self.smoothness,
                "detail_level": self.detail_level,
                "output_format": self.output_format,
                "generate_labels": self.generate_labels
            }
        }
        
        # Add num_segments if specified
        if self.num_segments is not None:
            payload["options"]["num_segments"] = self.num_segments
        
        self.logger.info(f"Creating Tripo segmentation task with payload: {payload}")
        
        try:
            async with httpx.AsyncClient(timeout=60.0, trust_env=False) as client:
                response = await client.post(
                    self.base_url,
                    headers=headers,
                    json=payload
                )
                
                if response.status_code != 200:
                    # Enhanced error handling
                    if response.status_code == 400:
                        try:
                            error_data = response.json()
                            error_code = error_data.get("code")
                            error_msg = error_data.get("message", "Unknown error")
                            
                            if error_code == 1004:  # Parameter invalid
                                error_msg = (
                                    f"Tripo segmentation parameter error: {error_msg}. "
                                    "This may indicate that mesh segmentation is not available on your account, "
                                    "or the task type/parameters are incorrect. "
                                    "Please check your account capabilities at https://platform.tripo3d.ai"
                                )
                            elif error_code == 2002:  # Task type not supported
                                error_msg = (
                                    f"Tripo segmentation not supported: {error_msg}. "
                                    "Mesh segmentation may not be available on your subscription plan."
                                )
                            else:
                                error_msg = f"Tripo segmentation error {error_code}: {error_msg}"
                                
                        except:
                            error_msg = f"Tripo segmentation error 400: {response.text}"
                    else:
                        error_msg = f"Tripo segmentation error {response.status_code}: {response.text}"
                    
                    raise ProcessingError(error_msg)
                
                result = response.json()
                self.logger.debug(f"Segmentation API response: {result}")
                
                # Handle task creation response
                if "data" in result and "task_id" in result["data"]:
                    task_id = result["data"]["task_id"]
                    self.logger.info(f"Created Tripo segmentation task: {task_id}")
                    return await self._poll_segmentation_task(task_id)
                elif "task_id" in result:
                    task_id = result["task_id"]
                    self.logger.info(f"Created Tripo segmentation task: {task_id}")
                    return await self._poll_segmentation_task(task_id)
                else:
                    raise ProcessingError(f"Unexpected Tripo segmentation response: {result}")
                
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to create Tripo segmentation task: {e}")
    
    async def _poll_segmentation_task(self, task_id: str) -> Dict[str, Any]:
        """Poll segmentation task status until completion.
        
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
                    # Check timeout
                    if time.time() - start_time > self.max_wait_time:
                        raise ProcessingError(f"Tripo segmentation task {task_id} timed out after {self.max_wait_time}s")
                    
                    # Get task status
                    response = await client.get(poll_url, headers=headers)
                    
                    if response.status_code != 200:
                        error_msg = f"Failed to get segmentation task status {response.status_code}: {response.text}"
                        raise ProcessingError(error_msg)
                    
                    result = response.json()
                    
                    if "data" in result:
                        data = result["data"]
                        status = data.get("status")
                        progress = data.get("progress", 0)
                        
                        self.logger.info(f"Tripo segmentation task {task_id} status: {status} ({progress}%)")
                        
                        if status == "success":
                            return data
                        elif status == "failed":
                            error = data.get("error", "Unknown segmentation error")
                            raise ProcessingError(f"Tripo segmentation task failed: {error}")
                        elif status in ["queued", "running"]:
                            await asyncio.sleep(self.poll_interval)
                        else:
                            self.logger.warning(f"Unknown Tripo segmentation status: {status}")
                            await asyncio.sleep(self.poll_interval)
                    else:
                        raise ProcessingError(f"Unexpected segmentation polling response: {result}")
                        
        except httpx.RequestError as e:
            raise ProcessingError(f"Failed to poll Tripo segmentation task: {e}")
    
    async def _download_segmented_parts(
        self, 
        task_result: Dict[str, Any], 
        hero: Hero
    ) -> tuple[List[Path], List[str]]:
        """Download segmented parts from task result.
        
        Args:
            task_result: Completed segmentation task result.
            hero: Hero for output paths.
            
        Returns:
            Tuple of (parts_paths, part_names).
        """
        parts_paths = []
        part_names = []
        
        # Create parts directory
        parts_dir = hero.hero_dir / "parts"
        parts_dir.mkdir(exist_ok=True)
        
        # Extract segmentation results
        # Note: The exact response format may vary - this is based on typical patterns
        segments = []
        
        # Try different possible result structures
        if "result" in task_result:
            result = task_result["result"]
            if "segments" in result:
                segments = result["segments"]
            elif "parts" in result:
                segments = result["parts"]
            elif isinstance(result, list):
                segments = result
        elif "segments" in task_result:
            segments = task_result["segments"]
        elif "parts" in task_result:
            segments = task_result["parts"]
        
        if not segments:
            raise ProcessingError(f"No segments found in Tripo segmentation result: {list(task_result.keys())}")
        
        # Download each segment
        async with httpx.AsyncClient(timeout=300.0, trust_env=False) as client:
            for i, segment in enumerate(segments):
                try:
                    # Extract segment info
                    if isinstance(segment, dict):
                        segment_url = segment.get("url") or segment.get("mesh_url") or segment.get("file_url")
                        segment_label = segment.get("label") or segment.get("name") or f"part_{i+1}"
                    else:
                        # If segments is just a list of URLs
                        segment_url = segment
                        segment_label = f"part_{i+1}"
                    
                    if not segment_url:
                        self.logger.warning(f"No URL found for segment {i+1}, skipping")
                        continue
                    
                    # Determine file extension
                    if segment_url.endswith('.obj'):
                        file_extension = ".obj"
                    elif segment_url.endswith('.glb'):
                        file_extension = ".glb"
                    elif segment_url.endswith('.ply'):
                        file_extension = ".ply"
                    else:
                        file_extension = f".{self.output_format}"
                    
                    # Clean label for filename
                    clean_label = "".join(c for c in segment_label if c.isalnum() or c in "._-")
                    part_filename = f"{clean_label}{file_extension}"
                    part_path = parts_dir / part_filename
                    
                    # Download segment
                    self.logger.info(f"Downloading segment {i+1}: {segment_label}")
                    response = await client.get(segment_url)
                    
                    if response.status_code != 200:
                        self.logger.warning(f"Failed to download segment {i+1}: {response.status_code}")
                        continue
                    
                    with open(part_path, 'wb') as f:
                        f.write(response.content)
                    
                    parts_paths.append(part_path)
                    part_names.append(segment_label)
                    
                    self.logger.info(f"Downloaded part: {part_path} (size: {part_path.stat().st_size} bytes)")
                    
                except Exception as e:
                    self.logger.warning(f"Failed to download segment {i+1}: {e}")
                    continue
        
        if not parts_paths:
            raise ProcessingError("Failed to download any segmented parts from Tripo result")
        
        self.logger.info(f"Successfully downloaded {len(parts_paths)} parts")
        return parts_paths, part_names
    
    def validate_inputs(self, hero: Hero) -> bool:
        """Validate inputs for Tripo segmentation.
        
        Args:
            hero: Hero to validate.
            
        Returns:
            True if inputs are valid.
        """
        # Call parent validation
        super().validate_inputs(hero)
        
        # Additional Tripo-specific validation
        mesh_path = hero.assets.get_asset(AssetType.MESH_3D)
        
        # Check file size (Tripo may have limits)
        file_size = mesh_path.stat().st_size
        max_size = 100 * 1024 * 1024  # 100MB limit (conservative estimate)
        if file_size > max_size:
            raise ValidationError(f"Mesh file too large for Tripo API: {file_size} bytes > {max_size} bytes")
        
        # Check mesh format
        suffix = mesh_path.suffix.lower()
        supported_formats = ['.obj', '.glb', '.gltf', '.ply', '.stl']
        if suffix not in supported_formats:
            raise ValidationError(f"Unsupported mesh format for Tripo segmentation: {suffix}. Supported: {supported_formats}")
        
        return True
"""Hero data model and related classes."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Dict, Any, List
from enum import Enum
import uuid
from datetime import datetime

from pydantic import BaseModel, Field, field_validator


class HeroStatus(str, Enum):
    """Status of hero processing pipeline."""
    CREATED = "created"
    PROCESSING = "processing"  
    READY = "ready"
    FAILED = "failed"


class AssetType(str, Enum):
    """Types of hero assets."""
    INPUT_IMAGE = "input_image"
    MESH_3D = "mesh_3d"
    PARTS = "parts"
    SKELETON = "skeleton"
    URDF = "urdf"
    SIMULATION_MODEL = "simulation_model"
    TRAINED_POLICY = "trained_policy"


@dataclass
class HeroAssets:
    """Container for hero assets at different pipeline stages."""
    
    input_image: Optional[Path] = None
    mesh_3d: Optional[Path] = None
    parts: Optional[List[Path]] = None
    skeleton: Optional[Path] = None 
    urdf: Optional[Path] = None
    simulation_model: Optional[Path] = None
    trained_policy: Optional[Path] = None
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def get_asset(self, asset_type: AssetType) -> Optional[Path]:
        """Get asset by type.
        
        Args:
            asset_type: Type of asset to retrieve.
            
        Returns:
            Path to asset file if exists, None otherwise.
        """
        return getattr(self, asset_type.value, None)
    
    def set_asset(self, asset_type: AssetType, path: Path) -> None:
        """Set asset path by type.
        
        Args:
            asset_type: Type of asset to set.
            path: Path to asset file.
        """
        setattr(self, asset_type.value, path)
    
    def has_asset(self, asset_type: AssetType) -> bool:
        """Check if asset exists.
        
        Args:
            asset_type: Type of asset to check.
            
        Returns:
            True if asset exists and file is present, False otherwise.
        """
        asset_path = self.get_asset(asset_type)
        return asset_path is not None and asset_path.exists()


class Hero(BaseModel):
    """Data model for a hero character."""
    
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    name: str = Field(..., description="Unique name for the hero")
    description: Optional[str] = None
    status: HeroStatus = HeroStatus.CREATED
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    
    # Asset management
    assets: HeroAssets = Field(default_factory=HeroAssets)
    hero_dir: Path = Field(..., description="Directory containing hero files")
    
    # Processing configuration
    config: Dict[str, Any] = Field(default_factory=dict)
    
    # Processing history
    processing_log: List[Dict[str, Any]] = Field(default_factory=list)
    
    class Config:
        """Pydantic configuration."""
        arbitrary_types_allowed = True
        json_encoders = {
            Path: str,
            datetime: lambda v: v.isoformat(),
        }
    
    @field_validator('hero_dir')
    @classmethod
    def validate_hero_dir(cls, v):
        """Validate hero directory exists."""
        if not v.exists():
            v.mkdir(parents=True, exist_ok=True)
        return v
    
    @field_validator('name')
    @classmethod
    def validate_name(cls, v):
        """Validate hero name is not empty."""
        if not v.strip():
            raise ValueError("Hero name cannot be empty")
        return v.strip()
    
    def update_status(self, status: HeroStatus, message: Optional[str] = None) -> None:
        """Update hero status and log the change.
        
        Args:
            status: New status to set.
            message: Optional message describing the status change.
        """
        old_status = self.status
        self.status = status
        self.updated_at = datetime.now()
        
        log_entry = {
            "timestamp": self.updated_at,
            "status_change": {"from": old_status, "to": status},
            "message": message
        }
        self.processing_log.append(log_entry)
    
    def add_processing_log(self, stage: str, message: str, details: Optional[Dict[str, Any]] = None) -> None:
        """Add an entry to the processing log.
        
        Args:
            stage: Pipeline stage name.
            message: Log message.
            details: Optional additional details.
        """
        log_entry = {
            "timestamp": datetime.now(),
            "stage": stage,
            "message": message,
            "details": details or {}
        }
        self.processing_log.append(log_entry)
        self.updated_at = datetime.now()
    
    def get_config_value(self, key: str, default: Any = None) -> Any:
        """Get configuration value with dot notation support.
        
        Args:
            key: Configuration key (supports dot notation like 'generation.model').
            default: Default value if key not found.
            
        Returns:
            Configuration value or default.
        """
        keys = key.split('.')
        value = self.config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def set_config_value(self, key: str, value: Any) -> None:
        """Set configuration value with dot notation support.
        
        Args:
            key: Configuration key (supports dot notation).
            value: Value to set.
        """
        keys = key.split('.')
        config = self.config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        config[keys[-1]] = value
        self.updated_at = datetime.now()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert hero to dictionary representation."""
        return self.model_dump()
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Hero':
        """Create hero from dictionary representation.
        
        Args:
            data: Dictionary containing hero data.
            
        Returns:
            Hero instance.
        """
        # Convert string paths back to Path objects
        if 'hero_dir' in data:
            data['hero_dir'] = Path(data['hero_dir'])
        
        if 'assets' in data:
            assets_data = data['assets']
            assets = HeroAssets()
            for key, value in assets_data.items():
                if key != 'metadata' and value is not None:
                    if isinstance(value, list):
                        setattr(assets, key, [Path(p) for p in value])
                    else:
                        setattr(assets, key, Path(value))
                elif key == 'metadata':
                    assets.metadata = value
            data['assets'] = assets
        
        return cls(**data)
    
    def save(self, path: Optional[Path] = None) -> Path:
        """Save hero metadata to JSON file.
        
        Args:
            path: Optional path to save to. If None, saves to hero_dir/hero.json.
            
        Returns:
            Path where hero was saved.
        """
        import json
        
        if path is None:
            path = self.hero_dir / "hero.json"
        
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f, indent=2, default=str)
        
        return path
    
    @classmethod
    def load(cls, path: Path) -> 'Hero':
        """Load hero from JSON file.
        
        Args:
            path: Path to hero JSON file.
            
        Returns:
            Hero instance.
        """
        import json
        
        with open(path, 'r') as f:
            data = json.load(f)
        
        return cls.from_dict(data)
"""Configuration management utilities."""

from pathlib import Path
from typing import Any, Dict, Optional, Union
import yaml
import json
import logging
from dataclasses import dataclass, field

from ..core.exceptions import ConfigurationError

logger = logging.getLogger(__name__)


@dataclass
class ConfigManager:
    """Manages configuration loading, merging, and validation."""
    
    # Configuration search paths
    search_paths: list[Path] = field(default_factory=lambda: [
        Path.cwd() / "config",
        Path.home() / ".tung_playground",
        Path("/etc/tung_playground"),
    ])
    
    # Loaded configuration
    _config: Dict[str, Any] = field(default_factory=dict)
    _config_files: list[Path] = field(default_factory=list)
    
    def load_config(
        self, 
        config_name: str = "default", 
        config_dir: Optional[Path] = None,
        merge_user_config: bool = True
    ) -> Dict[str, Any]:
        """Load configuration from YAML or JSON files.
        
        Args:
            config_name: Name of the configuration file (without extension).
            config_dir: Optional specific directory to load from.
            merge_user_config: Whether to merge user-specific configuration.
            
        Returns:
            Loaded and merged configuration dictionary.
            
        Raises:
            ConfigurationError: If configuration cannot be loaded.
        """
        config = {}
        loaded_files = []
        
        # Determine search paths
        if config_dir:
            search_paths = [config_dir]
        else:
            search_paths = self.search_paths
        
        # Try to load base configuration
        for search_path in search_paths:
            for ext in ['.yaml', '.yml', '.json']:
                config_file = search_path / f"{config_name}{ext}"
                if config_file.exists():
                    try:
                        file_config = self._load_file(config_file)
                        config = self._deep_merge(config, file_config)
                        loaded_files.append(config_file)
                        logger.info(f"Loaded configuration from {config_file}")
                        break
                    except Exception as e:
                        logger.error(f"Error loading config from {config_file}: {e}")
                        raise ConfigurationError(f"Failed to load config from {config_file}: {e}")
        
        # Load user-specific configuration if requested
        if merge_user_config:
            user_config_name = f"{config_name}.user"
            for search_path in search_paths:
                for ext in ['.yaml', '.yml', '.json']:
                    user_config_file = search_path / f"{user_config_name}{ext}"
                    if user_config_file.exists():
                        try:
                            user_config = self._load_file(user_config_file)
                            config = self._deep_merge(config, user_config)
                            loaded_files.append(user_config_file)
                            logger.info(f"Merged user configuration from {user_config_file}")
                            break
                        except Exception as e:
                            logger.warning(f"Error loading user config from {user_config_file}: {e}")
        
        if not loaded_files:
            raise ConfigurationError(f"No configuration files found for '{config_name}'")
        
        self._config = config
        self._config_files = loaded_files
        
        return config
    
    def _load_file(self, file_path: Path) -> Dict[str, Any]:
        """Load configuration from a single file.
        
        Args:
            file_path: Path to configuration file.
            
        Returns:
            Configuration dictionary.
            
        Raises:
            ConfigurationError: If file cannot be loaded.
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                if file_path.suffix.lower() == '.json':
                    return json.load(f)
                else:  # YAML
                    return yaml.safe_load(f) or {}
        except Exception as e:
            raise ConfigurationError(f"Failed to parse {file_path}: {e}")
    
    def _deep_merge(self, base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        """Deep merge two dictionaries.
        
        Args:
            base: Base dictionary.
            override: Dictionary to merge into base.
            
        Returns:
            Merged dictionary.
        """
        result = base.copy()
        
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = value
        
        return result
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value using dot notation.
        
        Args:
            key: Configuration key (supports dot notation like 'database.host').
            default: Default value if key not found.
            
        Returns:
            Configuration value or default.
        """
        keys = key.split('.')
        value = self._config
        
        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default
        
        return value
    
    def set(self, key: str, value: Any) -> None:
        """Set configuration value using dot notation.
        
        Args:
            key: Configuration key (supports dot notation).
            value: Value to set.
        """
        keys = key.split('.')
        config = self._config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
        
        config[keys[-1]] = value
    
    def save_config(self, file_path: Path, format: str = 'yaml') -> None:
        """Save current configuration to file.
        
        Args:
            file_path: Path to save configuration.
            format: File format ('yaml' or 'json').
            
        Raises:
            ConfigurationError: If configuration cannot be saved.
        """
        try:
            file_path.parent.mkdir(parents=True, exist_ok=True)
            
            with open(file_path, 'w', encoding='utf-8') as f:
                if format.lower() == 'json':
                    json.dump(self._config, f, indent=2, default=str)
                else:  # YAML
                    yaml.dump(self._config, f, default_flow_style=False, indent=2)
            
            logger.info(f"Configuration saved to {file_path}")
            
        except Exception as e:
            raise ConfigurationError(f"Failed to save configuration to {file_path}: {e}")
    
    def get_loaded_files(self) -> list[Path]:
        """Get list of configuration files that were loaded.
        
        Returns:
            List of loaded configuration file paths.
        """
        return self._config_files.copy()
    
    def reload(self) -> Dict[str, Any]:
        """Reload configuration from the same files.
        
        Returns:
            Reloaded configuration dictionary.
        """
        if self._config_files:
            # Get the base config name from the first loaded file
            base_file = self._config_files[0]
            config_name = base_file.stem
            config_dir = base_file.parent
            return self.load_config(config_name, config_dir)
        else:
            logger.warning("No configuration files to reload")
            return self._config
    
    @property
    def config(self) -> Dict[str, Any]:
        """Get the current configuration dictionary."""
        return self._config.copy()


# Global configuration manager instance
_global_config_manager = ConfigManager()


def load_config(
    config_name: str = "default",
    config_dir: Optional[Path] = None,
    merge_user_config: bool = True
) -> Dict[str, Any]:
    """Load configuration using the global configuration manager.
    
    Args:
        config_name: Name of the configuration file.
        config_dir: Optional specific directory to load from.
        merge_user_config: Whether to merge user-specific configuration.
        
    Returns:
        Loaded configuration dictionary.
    """
    return _global_config_manager.load_config(config_name, config_dir, merge_user_config)


def save_config(config: Dict[str, Any], file_path: Path, format: str = 'yaml') -> None:
    """Save configuration to file.
    
    Args:
        config: Configuration dictionary to save.
        file_path: Path to save configuration.
        format: File format ('yaml' or 'json').
    """
    # Temporarily update global config and save
    old_config = _global_config_manager._config
    _global_config_manager._config = config
    try:
        _global_config_manager.save_config(file_path, format)
    finally:
        _global_config_manager._config = old_config


def get_config_value(key: str, default: Any = None) -> Any:
    """Get configuration value from global configuration.
    
    Args:
        key: Configuration key (supports dot notation).
        default: Default value if key not found.
        
    Returns:
        Configuration value or default.
    """
    return _global_config_manager.get(key, default)


def set_config_value(key: str, value: Any) -> None:
    """Set configuration value in global configuration.
    
    Args:
        key: Configuration key (supports dot notation).
        value: Value to set.
    """
    _global_config_manager.set(key, value)
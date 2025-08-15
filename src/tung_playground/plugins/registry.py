"""Plugin registry system for dynamic component loading."""

import importlib
import inspect
from pathlib import Path
from typing import Dict, Type, Any, Optional, List, Callable, TypeVar, Union
import logging
from dataclasses import dataclass, field

from ..core.exceptions import PluginError, TungPlaygroundError
from ..core.pipeline import PipelineStage

logger = logging.getLogger(__name__)

T = TypeVar('T')


@dataclass
class PluginInfo:
    """Information about a registered plugin."""
    
    name: str
    plugin_type: str
    class_type: Type
    module_path: str
    description: Optional[str] = None
    version: Optional[str] = None
    author: Optional[str] = None
    dependencies: List[str] = field(default_factory=list)
    config_schema: Optional[Dict[str, Any]] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


class PluginRegistry:
    """Registry for managing plugins across different categories."""
    
    def __init__(self):
        """Initialize the plugin registry."""
        self._plugins: Dict[str, Dict[str, PluginInfo]] = {}
        self._instances: Dict[str, Any] = {}
        self.logger = logging.getLogger(__name__)
    
    def register(
        self,
        name: str,
        plugin_type: str,
        class_type: Type[T],
        description: Optional[str] = None,
        version: Optional[str] = None,
        author: Optional[str] = None,
        dependencies: Optional[List[str]] = None,
        config_schema: Optional[Dict[str, Any]] = None,
        **metadata
    ) -> None:
        """Register a plugin.
        
        Args:
            name: Unique name for the plugin within its type.
            plugin_type: Category of plugin (e.g., 'generation', 'decomposition').
            class_type: Class type of the plugin.
            description: Optional description of the plugin.
            version: Optional version string.
            author: Optional author information.
            dependencies: Optional list of required dependencies.
            config_schema: Optional configuration schema.
            **metadata: Additional metadata.
            
        Raises:
            PluginError: If plugin registration fails.
        """
        # Validate plugin type is a pipeline stage
        if plugin_type in ['generation', 'decomposition', 'rigging', 'urdf', 'simulation', 'training']:
            if not issubclass(class_type, PipelineStage):
                raise PluginError(f"Plugin {name} of type {plugin_type} must inherit from PipelineStage")
        
        # Initialize plugin type category if needed
        if plugin_type not in self._plugins:
            self._plugins[plugin_type] = {}
        
        # Check for existing plugin
        if name in self._plugins[plugin_type]:
            raise PluginError(f"Plugin '{name}' of type '{plugin_type}' is already registered")
        
        # Create plugin info
        plugin_info = PluginInfo(
            name=name,
            plugin_type=plugin_type,
            class_type=class_type,
            module_path=class_type.__module__,
            description=description,
            version=version,
            author=author,
            dependencies=dependencies or [],
            config_schema=config_schema,
            metadata=metadata
        )
        
        # Register the plugin
        self._plugins[plugin_type][name] = plugin_info
        
        self.logger.info(f"Registered plugin: {plugin_type}.{name} ({class_type.__name__})")
    
    def get(self, plugin_type: str, name: str) -> Type:
        """Get a plugin class by type and name.
        
        Args:
            plugin_type: Type of plugin.
            name: Name of plugin.
            
        Returns:
            Plugin class.
            
        Raises:
            PluginError: If plugin is not found.
        """
        if plugin_type not in self._plugins:
            raise PluginError(f"No plugins registered for type '{plugin_type}'")
        
        if name not in self._plugins[plugin_type]:
            available = list(self._plugins[plugin_type].keys())
            raise PluginError(f"Plugin '{name}' not found in type '{plugin_type}'. Available: {available}")
        
        return self._plugins[plugin_type][name].class_type
    
    def get_info(self, plugin_type: str, name: str) -> PluginInfo:
        """Get plugin information.
        
        Args:
            plugin_type: Type of plugin.
            name: Name of plugin.
            
        Returns:
            Plugin information.
            
        Raises:
            PluginError: If plugin is not found.
        """
        if plugin_type not in self._plugins:
            raise PluginError(f"No plugins registered for type '{plugin_type}'")
        
        if name not in self._plugins[plugin_type]:
            available = list(self._plugins[plugin_type].keys())
            raise PluginError(f"Plugin '{name}' not found in type '{plugin_type}'. Available: {available}")
        
        return self._plugins[plugin_type][name]
    
    def create_instance(
        self,
        plugin_type: str,
        name: str,
        config: Optional[Dict[str, Any]] = None,
        cache: bool = True,
        **kwargs
    ) -> Any:
        """Create an instance of a plugin.
        
        Args:
            plugin_type: Type of plugin.
            name: Name of plugin.
            config: Optional configuration dictionary.
            cache: Whether to cache the instance.
            **kwargs: Additional arguments for plugin constructor.
            
        Returns:
            Plugin instance.
            
        Raises:
            PluginError: If plugin creation fails.
        """
        cache_key = f"{plugin_type}.{name}"
        
        # Return cached instance if available
        if cache and cache_key in self._instances:
            return self._instances[cache_key]
        
        # Get plugin class
        plugin_class = self.get(plugin_type, name)
        plugin_info = self.get_info(plugin_type, name)
        
        # Check dependencies
        self._check_dependencies(plugin_info)
        
        try:
            # Create instance
            if issubclass(plugin_class, PipelineStage):
                # Pipeline stages need name and config
                instance = plugin_class(name=name, config=config or {}, **kwargs)
            else:
                # Other plugins
                instance = plugin_class(config=config or {}, **kwargs)
            
            # Cache instance if requested
            if cache:
                self._instances[cache_key] = instance
            
            self.logger.info(f"Created plugin instance: {cache_key}")
            return instance
            
        except Exception as e:
            raise PluginError(f"Failed to create plugin instance '{cache_key}': {e}")
    
    def _check_dependencies(self, plugin_info: PluginInfo) -> None:
        """Check if plugin dependencies are available.
        
        Args:
            plugin_info: Plugin information to check.
            
        Raises:
            PluginError: If dependencies are missing.
        """
        missing_deps = []
        
        for dep in plugin_info.dependencies:
            try:
                importlib.import_module(dep)
            except ImportError:
                missing_deps.append(dep)
        
        if missing_deps:
            raise PluginError(
                f"Plugin '{plugin_info.name}' has missing dependencies: {missing_deps}"
            )
    
    def list_plugins(self, plugin_type: Optional[str] = None) -> Dict[str, List[str]]:
        """List all registered plugins.
        
        Args:
            plugin_type: Optional plugin type to filter by.
            
        Returns:
            Dictionary mapping plugin types to lists of plugin names.
        """
        if plugin_type:
            if plugin_type in self._plugins:
                return {plugin_type: list(self._plugins[plugin_type].keys())}
            else:
                return {plugin_type: []}
        
        return {
            ptype: list(plugins.keys())
            for ptype, plugins in self._plugins.items()
        }
    
    def list_plugin_types(self) -> List[str]:
        """List all registered plugin types.
        
        Returns:
            List of plugin type names.
        """
        return list(self._plugins.keys())
    
    def unregister(self, plugin_type: str, name: str) -> bool:
        """Unregister a plugin.
        
        Args:
            plugin_type: Type of plugin.
            name: Name of plugin.
            
        Returns:
            True if plugin was unregistered, False if not found.
        """
        if plugin_type not in self._plugins:
            return False
        
        if name not in self._plugins[plugin_type]:
            return False
        
        # Remove from registry
        del self._plugins[plugin_type][name]
        
        # Remove cached instance
        cache_key = f"{plugin_type}.{name}"
        if cache_key in self._instances:
            del self._instances[cache_key]
        
        self.logger.info(f"Unregistered plugin: {plugin_type}.{name}")
        return True
    
    def clear_cache(self) -> None:
        """Clear all cached plugin instances."""
        self._instances.clear()
        self.logger.info("Cleared plugin instance cache")
    
    def clear_type(self, plugin_type: str) -> None:
        """Clear all plugins of a specific type.
        
        Args:
            plugin_type: Type of plugins to clear.
        """
        if plugin_type in self._plugins:
            # Clear cached instances
            for name in self._plugins[plugin_type]:
                cache_key = f"{plugin_type}.{name}"
                if cache_key in self._instances:
                    del self._instances[cache_key]
            
            # Clear registry
            del self._plugins[plugin_type]
            
            self.logger.info(f"Cleared all plugins of type: {plugin_type}")
    
    def auto_discover(self, search_paths: List[Union[str, Path]], pattern: str = "*.py") -> int:
        """Automatically discover and load plugins from specified paths.
        
        Args:
            search_paths: List of paths to search for plugins.
            pattern: File pattern to match.
            
        Returns:
            Number of plugins discovered and loaded.
        """
        discovered_count = 0
        
        for search_path in search_paths:
            path = Path(search_path)
            if not path.exists():
                continue
            
            # Find Python files
            if path.is_file() and path.suffix == '.py':
                files = [path]
            else:
                files = list(path.rglob(pattern))
            
            for file_path in files:
                try:
                    # Import module
                    module_name = file_path.stem
                    spec = importlib.util.spec_from_file_location(module_name, file_path)
                    if spec and spec.loader:
                        module = importlib.util.module_from_spec(spec)
                        spec.loader.exec_module(module)
                        
                        # Look for plugin classes
                        for name, obj in inspect.getmembers(module, inspect.isclass):
                            if self._is_plugin_class(obj):
                                # Auto-register if not already registered
                                plugin_type = self._get_plugin_type(obj)
                                plugin_name = getattr(obj, 'PLUGIN_NAME', name)
                                
                                if plugin_type and plugin_name:
                                    try:
                                        self.register(
                                            name=plugin_name,
                                            plugin_type=plugin_type,
                                            class_type=obj,
                                            description=getattr(obj, '__doc__', None),
                                            version=getattr(obj, 'VERSION', None),
                                            author=getattr(obj, 'AUTHOR', None)
                                        )
                                        discovered_count += 1
                                    except PluginError:
                                        # Already registered
                                        pass
                        
                except Exception as e:
                    self.logger.warning(f"Failed to load plugin from {file_path}: {e}")
        
        self.logger.info(f"Auto-discovered {discovered_count} plugins")
        return discovered_count
    
    def _is_plugin_class(self, cls: Type) -> bool:
        """Check if a class is a valid plugin.
        
        Args:
            cls: Class to check.
            
        Returns:
            True if class is a valid plugin.
        """
        # Check if it's a pipeline stage
        try:
            return (
                inspect.isclass(cls) and
                issubclass(cls, PipelineStage) and
                cls != PipelineStage and
                not inspect.isabstract(cls)
            )
        except TypeError:
            return False
    
    def _get_plugin_type(self, cls: Type) -> Optional[str]:
        """Get plugin type from class.
        
        Args:
            cls: Plugin class.
            
        Returns:
            Plugin type string or None.
        """
        # Check for explicit plugin type
        if hasattr(cls, 'PLUGIN_TYPE'):
            return cls.PLUGIN_TYPE
        
        # Infer from module path
        module_path = cls.__module__
        if 'generation' in module_path:
            return 'generation'
        elif 'decomposition' in module_path:
            return 'decomposition'
        elif 'rigging' in module_path:
            return 'rigging'
        elif 'urdf' in module_path:
            return 'urdf'
        elif 'simulation' in module_path:
            return 'simulation'
        elif 'training' in module_path:
            return 'training'
        
        return None


# Global plugin registry instance
plugin_registry = PluginRegistry()


def register_plugin(
    name: str,
    plugin_type: str,
    class_type: Type[T],
    description: Optional[str] = None,
    version: Optional[str] = None,
    author: Optional[str] = None,
    dependencies: Optional[List[str]] = None,
    config_schema: Optional[Dict[str, Any]] = None,
    **metadata
) -> None:
    """Register a plugin with the global registry.
    
    Args:
        name: Unique name for the plugin.
        plugin_type: Category of plugin.
        class_type: Class type of the plugin.
        description: Optional description.
        version: Optional version string.
        author: Optional author information.
        dependencies: Optional list of dependencies.
        config_schema: Optional configuration schema.
        **metadata: Additional metadata.
    """
    plugin_registry.register(
        name=name,
        plugin_type=plugin_type,
        class_type=class_type,
        description=description,
        version=version,
        author=author,
        dependencies=dependencies,
        config_schema=config_schema,
        **metadata
    )


def get_plugin(plugin_type: str, name: str) -> Type:
    """Get a plugin class from the global registry.
    
    Args:
        plugin_type: Type of plugin.
        name: Name of plugin.
        
    Returns:
        Plugin class.
    """
    return plugin_registry.get(plugin_type, name)


def list_plugins(plugin_type: Optional[str] = None) -> Dict[str, List[str]]:
    """List plugins from the global registry.
    
    Args:
        plugin_type: Optional plugin type to filter by.
        
    Returns:
        Dictionary mapping plugin types to plugin names.
    """
    return plugin_registry.list_plugins(plugin_type)


def plugin(plugin_type: str, name: Optional[str] = None, **kwargs) -> Callable[[Type[T]], Type[T]]:
    """Decorator for registering plugins.
    
    Args:
        plugin_type: Type of plugin.
        name: Optional name (defaults to class name).
        **kwargs: Additional registration arguments.
        
    Returns:
        Decorator function.
    """
    def decorator(cls: Type[T]) -> Type[T]:
        plugin_name = name or cls.__name__
        register_plugin(
            name=plugin_name,
            plugin_type=plugin_type,
            class_type=cls,
            **kwargs
        )
        return cls
    
    return decorator
"""Plugin system for Tung Playground.

This module provides a flexible plugin architecture for extending pipeline stages
and adding new functionality to the framework.
"""

from .registry import PluginRegistry, plugin_registry, register_plugin, get_plugin, list_plugins

__all__ = [
    "PluginRegistry",
    "plugin_registry",
    "register_plugin",
    "get_plugin", 
    "list_plugins",
]
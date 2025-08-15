"""Logging utilities for Tung Playground."""

import logging
import sys
from pathlib import Path
from typing import Optional, Dict, Any
from rich.logging import RichHandler
from rich.console import Console


def setup_logging(
    level: str = "INFO",
    log_file: Optional[Path] = None,
    use_rich: bool = True,
    format_string: Optional[str] = None
) -> None:
    """Set up logging configuration.
    
    Args:
        level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
        log_file: Optional path to log file.
        use_rich: Whether to use rich formatting for console output.
        format_string: Custom format string for log messages.
    """
    # Convert string level to logging level
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    
    # Clear any existing handlers
    root_logger = logging.getLogger()
    root_logger.handlers.clear()
    
    # Set up formatters
    if format_string is None:
        format_string = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    
    formatter = logging.Formatter(format_string)
    
    # Set up console handler
    if use_rich:
        console_handler = RichHandler(
            console=Console(stderr=True),
            show_time=True,
            show_path=True,
            markup=True,
            rich_tracebacks=True
        )
        console_handler.setLevel(numeric_level)
    else:
        console_handler = logging.StreamHandler(sys.stderr)
        console_handler.setLevel(numeric_level)
        console_handler.setFormatter(formatter)
    
    root_logger.addHandler(console_handler)
    
    # Set up file handler if specified
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)
    
    # Set root logger level
    root_logger.setLevel(numeric_level)
    
    # Configure specific loggers
    configure_package_loggers(numeric_level)


def configure_package_loggers(level: int) -> None:
    """Configure loggers for specific packages.
    
    Args:
        level: Logging level to set.
    """
    # Tung playground loggers
    tung_logger = logging.getLogger("tung_playground")
    tung_logger.setLevel(level)
    
    # Reduce verbosity of third-party packages
    third_party_loggers = [
        "urllib3",
        "requests",
        "PIL",
        "matplotlib",
        "trimesh",
        "open3d"
    ]
    
    for logger_name in third_party_loggers:
        logger = logging.getLogger(logger_name)
        logger.setLevel(max(level, logging.WARNING))


def get_logger(name: str) -> logging.Logger:
    """Get a logger with the specified name.
    
    Args:
        name: Name of the logger.
        
    Returns:
        Configured logger instance.
    """
    return logging.getLogger(name)


class LoggerMixin:
    """Mixin class to add logging capabilities to any class."""
    
    @property
    def logger(self) -> logging.Logger:
        """Get logger for this class."""
        return logging.getLogger(f"{self.__class__.__module__}.{self.__class__.__name__}")


def log_function_call(func_name: str, args: tuple, kwargs: Dict[str, Any]) -> None:
    """Log a function call with its arguments.
    
    Args:
        func_name: Name of the function being called.
        args: Positional arguments.
        kwargs: Keyword arguments.
    """
    logger = logging.getLogger(__name__)
    
    # Format arguments for logging
    arg_strs = [repr(arg) for arg in args]
    kwarg_strs = [f"{k}={repr(v)}" for k, v in kwargs.items()]
    all_args = ", ".join(arg_strs + kwarg_strs)
    
    logger.debug(f"Calling {func_name}({all_args})")


def log_execution_time(func_name: str, execution_time: float) -> None:
    """Log execution time for a function.
    
    Args:
        func_name: Name of the function.
        execution_time: Execution time in seconds.
    """
    logger = logging.getLogger(__name__)
    logger.info(f"{func_name} completed in {execution_time:.3f}s")


def log_memory_usage(func_name: str, memory_before: float, memory_after: float) -> None:
    """Log memory usage for a function.
    
    Args:
        func_name: Name of the function.
        memory_before: Memory usage before function call in MB.
        memory_after: Memory usage after function call in MB.
    """
    logger = logging.getLogger(__name__)
    memory_diff = memory_after - memory_before
    logger.info(f"{func_name} memory usage: {memory_diff:+.2f} MB (before: {memory_before:.2f} MB, after: {memory_after:.2f} MB)")


class ProgressLogger:
    """Logger for tracking progress of long-running operations."""
    
    def __init__(self, name: str, total: int, log_interval: int = 10):
        """Initialize progress logger.
        
        Args:
            name: Name of the operation being tracked.
            total: Total number of items to process.
            log_interval: How often to log progress (every N items).
        """
        self.name = name
        self.total = total
        self.log_interval = log_interval
        self.current = 0
        self.logger = logging.getLogger(__name__)
        
        self.logger.info(f"Starting {self.name}: {self.total} items to process")
    
    def update(self, count: int = 1) -> None:
        """Update progress counter.
        
        Args:
            count: Number of items processed in this update.
        """
        self.current += count
        
        if self.current % self.log_interval == 0 or self.current == self.total:
            percentage = (self.current / self.total) * 100
            self.logger.info(f"{self.name}: {self.current}/{self.total} ({percentage:.1f}%)")
    
    def complete(self) -> None:
        """Mark operation as complete."""
        self.logger.info(f"{self.name} completed: {self.current}/{self.total} items processed")
"""Custom exceptions for Tung Playground."""

from typing import Optional, Any


class TungPlaygroundError(Exception):
    """Base exception for all Tung Playground errors."""
    
    def __init__(self, message: str, details: Optional[dict[str, Any]] = None):
        """Initialize the exception.
        
        Args:
            message: Human-readable error message.
            details: Optional dictionary with additional error details.
        """
        super().__init__(message)
        self.message = message
        self.details = details or {}
    
    def __str__(self) -> str:
        """Return string representation of the error."""
        if self.details:
            return f"{self.message} | Details: {self.details}"
        return self.message


class ValidationError(TungPlaygroundError):
    """Raised when validation fails during pipeline processing."""
    pass


class ProcessingError(TungPlaygroundError):
    """Raised when processing fails during pipeline execution."""
    pass


class ConfigurationError(TungPlaygroundError):
    """Raised when there are configuration-related issues."""
    pass


class PluginError(TungPlaygroundError):
    """Raised when there are plugin-related issues."""
    pass
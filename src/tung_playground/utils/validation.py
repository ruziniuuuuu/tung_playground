"""Validation utilities for Tung Playground."""

from pathlib import Path
from typing import List, Optional, Union, Any
import mimetypes
from PIL import Image
import logging

from ..core.exceptions import ValidationError

logger = logging.getLogger(__name__)


def validate_file_exists(file_path: Union[str, Path], description: str = "File") -> Path:
    """Validate that a file exists.
    
    Args:
        file_path: Path to the file to validate.
        description: Description of the file for error messages.
        
    Returns:
        Path object for the validated file.
        
    Raises:
        ValidationError: If file doesn't exist or is not a file.
    """
    path = Path(file_path)
    
    if not path.exists():
        raise ValidationError(f"{description} does not exist: {path}")
    
    if not path.is_file():
        raise ValidationError(f"{description} is not a file: {path}")
    
    return path


def validate_directory(dir_path: Union[str, Path], description: str = "Directory", create: bool = False) -> Path:
    """Validate that a directory exists.
    
    Args:
        dir_path: Path to the directory to validate.
        description: Description of the directory for error messages.
        create: Whether to create the directory if it doesn't exist.
        
    Returns:
        Path object for the validated directory.
        
    Raises:
        ValidationError: If directory doesn't exist and create=False, or if path exists but is not a directory.
    """
    path = Path(dir_path)
    
    if not path.exists():
        if create:
            path.mkdir(parents=True, exist_ok=True)
            logger.info(f"Created {description.lower()}: {path}")
        else:
            raise ValidationError(f"{description} does not exist: {path}")
    
    if not path.is_dir():
        raise ValidationError(f"{description} is not a directory: {path}")
    
    return path


def validate_image_file(image_path: Union[str, Path], description: str = "Image file") -> Path:
    """Validate that a file is a valid image.
    
    Args:
        image_path: Path to the image file to validate.
        description: Description of the image for error messages.
        
    Returns:
        Path object for the validated image file.
        
    Raises:
        ValidationError: If file is not a valid image.
    """
    path = validate_file_exists(image_path, description)
    
    # Check MIME type
    mime_type, _ = mimetypes.guess_type(str(path))
    if mime_type is None or not mime_type.startswith('image/'):
        raise ValidationError(f"{description} is not an image file: {path}")
    
    # Try to open with PIL to validate it's a proper image
    try:
        with Image.open(path) as img:
            img.verify()  # Verify that it's a valid image
    except Exception as e:
        raise ValidationError(f"{description} is not a valid image: {path} - {e}")
    
    return path


def validate_mesh_file(mesh_path: Union[str, Path], description: str = "Mesh file") -> Path:
    """Validate that a file is a supported mesh format.
    
    Args:
        mesh_path: Path to the mesh file to validate.
        description: Description of the mesh for error messages.
        
    Returns:
        Path object for the validated mesh file.
        
    Raises:
        ValidationError: If file is not a supported mesh format.
    """
    path = validate_file_exists(mesh_path, description)
    
    # Supported mesh formats
    supported_extensions = {'.obj', '.ply', '.stl', '.dae', '.glb', '.gltf', '.fbx', '.3ds'}
    
    if path.suffix.lower() not in supported_extensions:
        raise ValidationError(
            f"{description} has unsupported format: {path.suffix}. "
            f"Supported formats: {', '.join(sorted(supported_extensions))}"
        )
    
    return path


def validate_urdf_file(urdf_path: Union[str, Path], description: str = "URDF file") -> Path:
    """Validate that a file is a valid URDF.
    
    Args:
        urdf_path: Path to the URDF file to validate.
        description: Description of the URDF for error messages.
        
    Returns:
        Path object for the validated URDF file.
        
    Raises:
        ValidationError: If file is not a valid URDF.
    """
    path = validate_file_exists(urdf_path, description)
    
    if path.suffix.lower() != '.urdf':
        raise ValidationError(f"{description} must have .urdf extension: {path}")
    
    # Basic XML validation
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(path)
        root = tree.getroot()
        
        if root.tag != 'robot':
            raise ValidationError(f"{description} does not have 'robot' as root element: {path}")
            
    except ET.ParseError as e:
        raise ValidationError(f"{description} is not valid XML: {path} - {e}")
    except Exception as e:
        raise ValidationError(f"Error validating {description}: {path} - {e}")
    
    return path


def validate_config_dict(config: dict, required_keys: List[str], description: str = "Configuration") -> None:
    """Validate that a configuration dictionary has required keys.
    
    Args:
        config: Configuration dictionary to validate.
        required_keys: List of required keys.
        description: Description of the configuration for error messages.
        
    Raises:
        ValidationError: If required keys are missing.
    """
    missing_keys = [key for key in required_keys if key not in config]
    
    if missing_keys:
        raise ValidationError(
            f"{description} is missing required keys: {missing_keys}"
        )


def validate_positive_number(value: Union[int, float], name: str) -> Union[int, float]:
    """Validate that a value is a positive number.
    
    Args:
        value: Value to validate.
        name: Name of the value for error messages.
        
    Returns:
        The validated value.
        
    Raises:
        ValidationError: If value is not a positive number.
    """
    if not isinstance(value, (int, float)):
        raise ValidationError(f"{name} must be a number, got {type(value).__name__}")
    
    if value <= 0:
        raise ValidationError(f"{name} must be positive, got {value}")
    
    return value


def validate_string_not_empty(value: str, name: str) -> str:
    """Validate that a string is not empty.
    
    Args:
        value: String to validate.
        name: Name of the string for error messages.
        
    Returns:
        The validated string (stripped).
        
    Raises:
        ValidationError: If string is empty or not a string.
    """
    if not isinstance(value, str):
        raise ValidationError(f"{name} must be a string, got {type(value).__name__}")
    
    value = value.strip()
    if not value:
        raise ValidationError(f"{name} cannot be empty")
    
    return value


def validate_choice(value: Any, choices: List[Any], name: str) -> Any:
    """Validate that a value is one of the allowed choices.
    
    Args:
        value: Value to validate.
        choices: List of allowed choices.
        name: Name of the value for error messages.
        
    Returns:
        The validated value.
        
    Raises:
        ValidationError: If value is not in choices.
    """
    if value not in choices:
        raise ValidationError(f"{name} must be one of {choices}, got {value}")
    
    return value


def validate_list_not_empty(value: List[Any], name: str) -> List[Any]:
    """Validate that a list is not empty.
    
    Args:
        value: List to validate.
        name: Name of the list for error messages.
        
    Returns:
        The validated list.
        
    Raises:
        ValidationError: If value is not a list or is empty.
    """
    if not isinstance(value, list):
        raise ValidationError(f"{name} must be a list, got {type(value).__name__}")
    
    if not value:
        raise ValidationError(f"{name} cannot be empty")
    
    return value


def validate_range(value: Union[int, float], min_val: Optional[Union[int, float]], max_val: Optional[Union[int, float]], name: str) -> Union[int, float]:
    """Validate that a value is within a specified range.
    
    Args:
        value: Value to validate.
        min_val: Minimum allowed value (inclusive).
        max_val: Maximum allowed value (inclusive).
        name: Name of the value for error messages.
        
    Returns:
        The validated value.
        
    Raises:
        ValidationError: If value is outside the range.
    """
    if not isinstance(value, (int, float)):
        raise ValidationError(f"{name} must be a number, got {type(value).__name__}")
    
    if min_val is not None and value < min_val:
        raise ValidationError(f"{name} must be >= {min_val}, got {value}")
    
    if max_val is not None and value > max_val:
        raise ValidationError(f"{name} must be <= {max_val}, got {value}")
    
    return value


class Validator:
    """Reusable validator class for complex validation scenarios."""
    
    def __init__(self, name: str):
        """Initialize validator.
        
        Args:
            name: Name of the value being validated.
        """
        self.name = name
        self.errors: List[str] = []
    
    def require_file(self, path: Union[str, Path]) -> 'Validator':
        """Require that a file exists.
        
        Args:
            path: Path to validate.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_file_exists(path, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_directory(self, path: Union[str, Path], create: bool = False) -> 'Validator':
        """Require that a directory exists.
        
        Args:
            path: Path to validate.
            create: Whether to create directory if it doesn't exist.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_directory(path, self.name, create)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_image(self, path: Union[str, Path]) -> 'Validator':
        """Require that a file is a valid image.
        
        Args:
            path: Path to validate.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_image_file(path, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_positive(self, value: Union[int, float]) -> 'Validator':
        """Require that a value is positive.
        
        Args:
            value: Value to validate.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_positive_number(value, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_not_empty(self, value: str) -> 'Validator':
        """Require that a string is not empty.
        
        Args:
            value: String to validate.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_string_not_empty(value, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_choice(self, value: Any, choices: List[Any]) -> 'Validator':
        """Require that a value is one of the allowed choices.
        
        Args:
            value: Value to validate.
            choices: List of allowed choices.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_choice(value, choices, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def require_range(self, value: Union[int, float], min_val: Optional[Union[int, float]], max_val: Optional[Union[int, float]]) -> 'Validator':
        """Require that a value is within a range.
        
        Args:
            value: Value to validate.
            min_val: Minimum allowed value.
            max_val: Maximum allowed value.
            
        Returns:
            Self for method chaining.
        """
        try:
            validate_range(value, min_val, max_val, self.name)
        except ValidationError as e:
            self.errors.append(str(e))
        return self
    
    def validate(self) -> None:
        """Perform validation and raise error if any validations failed.
        
        Raises:
            ValidationError: If any validations failed.
        """
        if self.errors:
            error_msg = f"Validation failed for {self.name}:\n" + "\n".join(f"- {error}" for error in self.errors)
            raise ValidationError(error_msg)
    
    @property
    def is_valid(self) -> bool:
        """Check if all validations passed.
        
        Returns:
            True if no validation errors, False otherwise.
        """
        return len(self.errors) == 0
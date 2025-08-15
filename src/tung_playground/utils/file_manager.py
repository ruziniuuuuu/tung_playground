"""File management utilities for Tung Playground."""

import shutil
from pathlib import Path
from typing import Optional, List, Union
import logging
from contextlib import contextmanager
import hashlib
import json

from ..core.exceptions import TungPlaygroundError

logger = logging.getLogger(__name__)


class FileManager:
    """Centralized file management for hero assets and configurations."""
    
    def __init__(self, base_dir: Path):
        """Initialize file manager.
        
        Args:
            base_dir: Base directory for file operations.
        """
        self.base_dir = Path(base_dir)
        self.ensure_directory(self.base_dir)
    
    def ensure_directory(self, dir_path: Union[str, Path]) -> Path:
        """Ensure a directory exists, creating it if necessary.
        
        Args:
            dir_path: Directory path to ensure.
            
        Returns:
            Path object for the directory.
        """
        path = Path(dir_path)
        path.mkdir(parents=True, exist_ok=True)
        return path
    
    def copy_file(self, source: Union[str, Path], destination: Union[str, Path], preserve_metadata: bool = True) -> Path:
        """Copy a file to a new location.
        
        Args:
            source: Source file path.
            destination: Destination file path.
            preserve_metadata: Whether to preserve file metadata.
            
        Returns:
            Path to the copied file.
            
        Raises:
            TungPlaygroundError: If copy operation fails.
        """
        source_path = Path(source)
        dest_path = Path(destination)
        
        if not source_path.exists():
            raise TungPlaygroundError(f"Source file does not exist: {source_path}")
        
        # Ensure destination directory exists
        self.ensure_directory(dest_path.parent)
        
        try:
            if preserve_metadata:
                shutil.copy2(source_path, dest_path)
            else:
                shutil.copy(source_path, dest_path)
            
            logger.info(f"Copied file: {source_path} -> {dest_path}")
            return dest_path
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to copy file: {e}")
    
    def move_file(self, source: Union[str, Path], destination: Union[str, Path]) -> Path:
        """Move a file to a new location.
        
        Args:
            source: Source file path.
            destination: Destination file path.
            
        Returns:
            Path to the moved file.
            
        Raises:
            TungPlaygroundError: If move operation fails.
        """
        source_path = Path(source)
        dest_path = Path(destination)
        
        if not source_path.exists():
            raise TungPlaygroundError(f"Source file does not exist: {source_path}")
        
        # Ensure destination directory exists
        self.ensure_directory(dest_path.parent)
        
        try:
            shutil.move(str(source_path), str(dest_path))
            logger.info(f"Moved file: {source_path} -> {dest_path}")
            return dest_path
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to move file: {e}")
    
    def delete_file(self, file_path: Union[str, Path], missing_ok: bool = True) -> bool:
        """Delete a file.
        
        Args:
            file_path: Path to file to delete.
            missing_ok: Whether to ignore missing files.
            
        Returns:
            True if file was deleted, False if it didn't exist.
            
        Raises:
            TungPlaygroundError: If deletion fails.
        """
        path = Path(file_path)
        
        if not path.exists():
            if missing_ok:
                return False
            else:
                raise TungPlaygroundError(f"File does not exist: {path}")
        
        try:
            path.unlink()
            logger.info(f"Deleted file: {path}")
            return True
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to delete file: {e}")
    
    def delete_directory(self, dir_path: Union[str, Path], missing_ok: bool = True) -> bool:
        """Delete a directory and all its contents.
        
        Args:
            dir_path: Path to directory to delete.
            missing_ok: Whether to ignore missing directories.
            
        Returns:
            True if directory was deleted, False if it didn't exist.
            
        Raises:
            TungPlaygroundError: If deletion fails.
        """
        path = Path(dir_path)
        
        if not path.exists():
            if missing_ok:
                return False
            else:
                raise TungPlaygroundError(f"Directory does not exist: {path}")
        
        try:
            shutil.rmtree(path)
            logger.info(f"Deleted directory: {path}")
            return True
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to delete directory: {e}")
    
    def list_files(self, directory: Union[str, Path], pattern: str = "*", recursive: bool = False) -> List[Path]:
        """List files in a directory.
        
        Args:
            directory: Directory to search.
            pattern: File pattern to match.
            recursive: Whether to search recursively.
            
        Returns:
            List of matching file paths.
        """
        dir_path = Path(directory)
        
        if not dir_path.exists():
            return []
        
        if recursive:
            return list(dir_path.rglob(pattern))
        else:
            return list(dir_path.glob(pattern))
    
    def get_file_hash(self, file_path: Union[str, Path], algorithm: str = "sha256") -> str:
        """Calculate hash of a file.
        
        Args:
            file_path: Path to file.
            algorithm: Hash algorithm to use.
            
        Returns:
            Hexadecimal hash string.
            
        Raises:
            TungPlaygroundError: If hashing fails.
        """
        path = Path(file_path)
        
        if not path.exists():
            raise TungPlaygroundError(f"File does not exist: {path}")
        
        try:
            hash_obj = hashlib.new(algorithm)
            
            with open(path, 'rb') as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    hash_obj.update(chunk)
            
            return hash_obj.hexdigest()
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to hash file: {e}")
    
    def get_file_size(self, file_path: Union[str, Path]) -> int:
        """Get size of a file in bytes.
        
        Args:
            file_path: Path to file.
            
        Returns:
            File size in bytes.
            
        Raises:
            TungPlaygroundError: If file doesn't exist.
        """
        path = Path(file_path)
        
        if not path.exists():
            raise TungPlaygroundError(f"File does not exist: {path}")
        
        return path.stat().st_size
    
    def create_unique_filename(self, base_path: Union[str, Path], extension: str = "") -> Path:
        """Create a unique filename by appending a counter if needed.
        
        Args:
            base_path: Base path for the file.
            extension: File extension to add.
            
        Returns:
            Unique file path.
        """
        path = Path(base_path)
        
        if extension and not extension.startswith('.'):
            extension = '.' + extension
        
        if extension:
            path = path.with_suffix(extension)
        
        if not path.exists():
            return path
        
        # Find unique name by appending counter
        counter = 1
        stem = path.stem
        suffix = path.suffix
        parent = path.parent
        
        while True:
            new_name = f"{stem}_{counter}{suffix}"
            new_path = parent / new_name
            
            if not new_path.exists():
                return new_path
            
            counter += 1
    
    def save_metadata(self, metadata: dict, file_path: Union[str, Path]) -> Path:
        """Save metadata dictionary to a JSON file.
        
        Args:
            metadata: Metadata dictionary to save.
            file_path: Path to save metadata.
            
        Returns:
            Path to saved metadata file.
            
        Raises:
            TungPlaygroundError: If saving fails.
        """
        path = Path(file_path)
        self.ensure_directory(path.parent)
        
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2, default=str)
            
            logger.info(f"Saved metadata to: {path}")
            return path
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to save metadata: {e}")
    
    def load_metadata(self, file_path: Union[str, Path]) -> dict:
        """Load metadata dictionary from a JSON file.
        
        Args:
            file_path: Path to metadata file.
            
        Returns:
            Loaded metadata dictionary.
            
        Raises:
            TungPlaygroundError: If loading fails.
        """
        path = Path(file_path)
        
        if not path.exists():
            raise TungPlaygroundError(f"Metadata file does not exist: {path}")
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            
            logger.info(f"Loaded metadata from: {path}")
            return metadata
            
        except Exception as e:
            raise TungPlaygroundError(f"Failed to load metadata: {e}")
    
    @contextmanager
    def temporary_directory(self, prefix: str = "tmp_"):
        """Context manager for creating a temporary directory.
        
        Args:
            prefix: Prefix for temporary directory name.
            
        Yields:
            Path to temporary directory.
        """
        import tempfile
        
        temp_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=self.base_dir))
        
        try:
            yield temp_dir
        finally:
            if temp_dir.exists():
                shutil.rmtree(temp_dir)


# Global utility functions
def ensure_directory(dir_path: Union[str, Path]) -> Path:
    """Ensure a directory exists, creating it if necessary.
    
    Args:
        dir_path: Directory path to ensure.
        
    Returns:
        Path object for the directory.
    """
    path = Path(dir_path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def copy_file(source: Union[str, Path], destination: Union[str, Path], preserve_metadata: bool = True) -> Path:
    """Copy a file to a new location.
    
    Args:
        source: Source file path.
        destination: Destination file path.
        preserve_metadata: Whether to preserve file metadata.
        
    Returns:
        Path to the copied file.
        
    Raises:
        TungPlaygroundError: If copy operation fails.
    """
    source_path = Path(source)
    dest_path = Path(destination)
    
    if not source_path.exists():
        raise TungPlaygroundError(f"Source file does not exist: {source_path}")
    
    # Ensure destination directory exists
    ensure_directory(dest_path.parent)
    
    try:
        if preserve_metadata:
            shutil.copy2(source_path, dest_path)
        else:
            shutil.copy(source_path, dest_path)
        
        logger.info(f"Copied file: {source_path} -> {dest_path}")
        return dest_path
        
    except Exception as e:
        raise TungPlaygroundError(f"Failed to copy file: {e}")


def move_file(source: Union[str, Path], destination: Union[str, Path]) -> Path:
    """Move a file to a new location.
    
    Args:
        source: Source file path.
        destination: Destination file path.
        
    Returns:
        Path to the moved file.
        
    Raises:
        TungPlaygroundError: If move operation fails.
    """
    source_path = Path(source)
    dest_path = Path(destination)
    
    if not source_path.exists():
        raise TungPlaygroundError(f"Source file does not exist: {source_path}")
    
    # Ensure destination directory exists
    ensure_directory(dest_path.parent)
    
    try:
        shutil.move(str(source_path), str(dest_path))
        logger.info(f"Moved file: {source_path} -> {dest_path}")
        return dest_path
        
    except Exception as e:
        raise TungPlaygroundError(f"Failed to move file: {e}")
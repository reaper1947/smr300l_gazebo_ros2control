#!/usr/bin/env python3
"""
Map Manager Service
Centralized service for managing maps across the entire system.
- Upload new maps (PGM + YAML)
- Switch active map dynamically
- Update all configuration files automatically
"""

import rclpy
from rclpy.node import Node
from zone_nav_interfaces.srv import UploadMap, SetActiveMap, GetActiveMap
import os
import yaml
import base64
import shutil
from datetime import datetime


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        # Base paths
        self.workspace_root = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main')
        self.maps_dir = os.path.join(self.workspace_root, 'maps')
        self.config_dir = os.path.join(self.workspace_root, 'config')
        
        # Active map configuration file
        self.active_map_config = os.path.join(self.workspace_root, 'active_map_config.yaml')
        
        # Load or initialize active map config
        self.active_map_path = self._load_active_map_config()
        
        # Services
        self.upload_map_srv = self.create_service(
            UploadMap,
            '/map_manager/upload_map',
            self.upload_map_callback
        )
        
        self.set_active_map_srv = self.create_service(
            SetActiveMap,
            '/map_manager/set_active_map',
            self.set_active_map_callback
        )
        
        self.get_active_map_srv = self.create_service(
            GetActiveMap,
            '/map_manager/get_active_map',
            self.get_active_map_callback
        )
        
        self.get_logger().info('Map Manager Service started')
        self.get_logger().info(f'Maps directory: {self.maps_dir}')
        self.get_logger().info(f'Active map: {self.active_map_path}')
    
    def _load_active_map_config(self):
        """Load the active map configuration"""
        if os.path.exists(self.active_map_config):
            try:
                with open(self.active_map_config, 'r') as f:
                    config = yaml.safe_load(f) or {}
                return config.get('active_map', os.path.join(self.maps_dir, 'smr_map.yaml'))
            except Exception as e:
                self.get_logger().error(f'Failed to load active map config: {e}')
        
        # Default map
        return os.path.join(self.maps_dir, 'smr_map.yaml')
    
    def _save_active_map_config(self, map_path):
        """Save the active map configuration"""
        try:
            config = {'active_map': map_path}
            with open(self.active_map_config, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            self.active_map_path = map_path
            self.get_logger().info(f'Active map config saved: {map_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save active map config: {e}')
            return False
    
    def upload_map_callback(self, request, response):
        """Handle map upload request"""
        try:
            map_name = request.map_name
            
            # Validate map name
            if not map_name or '/' in map_name or '\\' in map_name:
                response.success = False
                response.message = 'Invalid map name'
                return response
            
            # Decode and save PGM file
            pgm_path = os.path.join(self.maps_dir, f'{map_name}.pgm')
            try:
                pgm_data = base64.b64decode(request.pgm_base64)
                with open(pgm_path, 'wb') as f:
                    f.write(pgm_data)
                self.get_logger().info(f'Saved PGM: {pgm_path}')
            except Exception as e:
                response.success = False
                response.message = f'Failed to save PGM file: {e}'
                return response
            
            # Parse and save YAML file
            yaml_path = os.path.join(self.maps_dir, f'{map_name}.yaml')
            try:
                # Parse YAML content
                yaml_data = yaml.safe_load(request.yaml_content)
                
                # Ensure image path is relative and correct
                yaml_data['image'] = f'{map_name}.pgm'
                
                with open(yaml_path, 'w') as f:
                    yaml.dump(yaml_data, f, default_flow_style=False)
                self.get_logger().info(f'Saved YAML: {yaml_path}')
            except Exception as e:
                response.success = False
                response.message = f'Failed to save YAML file: {e}'
                # Cleanup PGM if YAML failed
                if os.path.exists(pgm_path):
                    os.remove(pgm_path)
                return response
            
            response.success = True
            response.message = f'Map "{map_name}" uploaded successfully'
            response.map_path = yaml_path
            
            self.get_logger().info(f'✓ Map uploaded: {map_name}')
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f'Upload failed: {e}'
            self.get_logger().error(response.message)
            return response
    
    def set_active_map_callback(self, request, response):
        """Switch the active map and update all configurations"""
        try:
            new_map_path = request.map_yaml_path
            
            # Validate map exists
            if not os.path.exists(new_map_path):
                response.success = False
                response.message = f'Map file not found: {new_map_path}'
                return response
            
            # Backup original configs before updating
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Update all configuration files
            files_to_update = [
                os.path.join(self.config_dir, 'nav2_params.yaml'),
                os.path.join(self.config_dir, 'nav2_params_working.yaml'),
            ]
            
            updated_files = []
            
            for config_file in files_to_update:
                if not os.path.exists(config_file):
                    continue
                
                try:
                    # Backup
                    backup_path = f"{config_file}.backup_{timestamp}"
                    shutil.copy(config_file, backup_path)
                    
                    # Update
                    with open(config_file, 'r') as f:
                        content = f.read()
                    
                    # Replace yaml_filename parameter
                    import re
                    pattern = r'(yaml_filename:\s*["\']?)([^"\']+\.yaml)(["\']?)'
                    
                    def replacer(match):
                        return f'{match.group(1)}{new_map_path}{match.group(3)}'
                    
                    updated_content = re.sub(pattern, replacer, content)
                    
                    with open(config_file, 'w') as f:
                        f.write(updated_content)
                    
                    updated_files.append(os.path.basename(config_file))
                    self.get_logger().info(f'✓ Updated: {config_file}')
                    
                except Exception as e:
                    self.get_logger().error(f'Failed to update {config_file}: {e}')
            
            # Update active map config
            if not self._save_active_map_config(new_map_path):
                response.success = False
                response.message = 'Failed to save active map configuration'
                return response
            
            response.success = True
            response.message = f'Active map switched to {os.path.basename(new_map_path)}. Updated: {", ".join(updated_files)}. ⚠️ Restart Nav2 to apply changes.'
            
            self.get_logger().info(f'✓ Active map set to: {new_map_path}')
            self.get_logger().warn('⚠️ You must restart Nav2 for changes to take effect')
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to set active map: {e}'
            self.get_logger().error(response.message)
            return response
    
    def get_active_map_callback(self, request, response):
        """Get the currently active map path"""
        try:
            response.success = True
            response.map_yaml_path = self.active_map_path
            response.message = f'Active map: {os.path.basename(self.active_map_path)}'
            return response
        except Exception as e:
            response.success = False
            response.message = f'Failed to get active map: {e}'
            return response


def main(args=None):
    rclpy.init(args=args)
    map_manager = MapManager()
    
    try:
        rclpy.spin(map_manager)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            map_manager.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Already shut down


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Professional Map Editor for Warehouse Robots
- Draw obstacles and no-go zones
- Set speed limit zones
- Edit map metadata
- Export/import map layers
"""

import rclpy
from rclpy.node import Node
from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import threading
import yaml
import os
import json
import math
import base64
from PIL import Image, ImageDraw
import io
import numpy as np


class MapEditorServer(Node):
    def __init__(self):
        super().__init__('map_editor_server')
        
        # Map storage
        self.map_path = os.path.expanduser('/home/aun/Downloads/smr300l_gazebo_ros2control-main/maps/smr_map.yaml')
        self.map_info = self.load_map_info()
        self.original_map = None
        self.edited_map = None
        
        # Map layers (separate editable layers)
        self.layers = {
            'obstacles': [],      # Drawn obstacles
            'no_go_zones': [],    # Red no-go areas
            'slow_zones': [],     # Yellow slow-speed zones
            'restricted': []      # Orange restricted areas
        }
        
        self.layers_file = os.path.expanduser('~/map_layers.json')
        self.load_layers()
        
        self.get_logger().info('Map Editor Server started')
        self.get_logger().info(f'Map: {self.map_info.get("image_path", "Not found")}')
    
    def load_map_info(self):
        """Load map YAML configuration"""
        try:
            with open(self.map_path, 'r') as f:
                map_data = yaml.safe_load(f)
            
            image_path = map_data.get('image', '')
            if not os.path.isabs(image_path):
                map_dir = os.path.dirname(self.map_path)
                image_path = os.path.join(map_dir, image_path)
            
            # Load original map image
            if os.path.exists(image_path):
                self.original_map = Image.open(image_path).convert('L')  # Grayscale
                self.edited_map = self.original_map.copy()
            
            return {
                'image_path': image_path,
                'resolution': map_data.get('resolution', 0.05),
                'origin': map_data.get('origin', [0, 0, 0]),
                'negate': map_data.get('negate', 0),
                'occupied_thresh': map_data.get('occupied_thresh', 0.65),
                'free_thresh': map_data.get('free_thresh', 0.196)
            }
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            return {}
    
    def load_layers(self):
        """Load saved map layers"""
        if os.path.exists(self.layers_file):
            try:
                with open(self.layers_file, 'r') as f:
                    self.layers = json.load(f)
                self.get_logger().info(f'Loaded {sum(len(v) for v in self.layers.values())} layer objects')
            except Exception as e:
                self.get_logger().error(f'Failed to load layers: {e}')
    
    def save_layers(self):
        """Save map layers to file"""
        try:
            with open(self.layers_file, 'w') as f:
                json.dump(self.layers, f, indent=2)
            self.get_logger().info('Map layers saved')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save layers: {e}')
            return False
    
    def add_layer_object(self, layer_type, obj):
        """Add object to a layer"""
        if layer_type in self.layers:
            self.layers[layer_type].append(obj)
            self.save_layers()
            return True
        return False
    
    def clear_layer(self, layer_type):
        """Clear all objects from a layer"""
        if layer_type in self.layers:
            self.layers[layer_type] = []
            self.save_layers()
            return True
        return False
    
    def render_map_with_layers(self):
        """Render map with all layers overlaid"""
        if self.original_map is None:
            return None
        
        # Start with original map
        img = self.original_map.copy().convert('RGB')
        draw = ImageDraw.Draw(img, 'RGBA')
        
        resolution = self.map_info.get('resolution', 0.05)
        origin = self.map_info.get('origin', [0, 0, 0])
        height = img.height
        
        def world_to_pixel(wx, wy):
            px = int((wx - origin[0]) / resolution)
            py = int(height - (wy - origin[1]) / resolution)
            return (px, py)
        
        # Draw obstacles (black)
        for obj in self.layers.get('obstacles', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(0, 0, 0, 255))
            elif obj['type'] == 'circle':
                center = world_to_pixel(obj['x'], obj['y'])
                radius_px = int(obj['radius'] / resolution)
                bbox = [center[0] - radius_px, center[1] - radius_px,
                       center[0] + radius_px, center[1] + radius_px]
                draw.ellipse(bbox, fill=(0, 0, 0, 255))
            elif obj['type'] == 'polygon':
                points = [world_to_pixel(p['x'], p['y']) for p in obj['points']]
                draw.polygon(points, fill=(0, 0, 0, 255))
        
        # Draw no-go zones (red transparent)
        for obj in self.layers.get('no_go_zones', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 0, 0, 100), outline=(255, 0, 0, 255), width=2)
            elif obj['type'] == 'polygon':
                points = [world_to_pixel(p['x'], p['y']) for p in obj['points']]
                draw.polygon(points, fill=(255, 0, 0, 100), outline=(255, 0, 0, 255))
        
        # Draw slow zones (yellow transparent)
        for obj in self.layers.get('slow_zones', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 255, 0, 80), outline=(255, 200, 0, 255), width=2)
                # Draw speed limit text
                speed = obj.get('speed_limit', 0.2)
                center_x = (p1[0] + p2[0]) // 2
                center_y = (p1[1] + p2[1]) // 2
                draw.text((center_x, center_y), f"{speed}m/s", fill=(0, 0, 0, 255))
        
        # Draw restricted zones (orange transparent)
        for obj in self.layers.get('restricted', []):
            if obj['type'] == 'rectangle':
                p1 = world_to_pixel(obj['x1'], obj['y1'])
                p2 = world_to_pixel(obj['x2'], obj['y2'])
                draw.rectangle([p1, p2], fill=(255, 165, 0, 100), outline=(255, 140, 0, 255), width=2)
        
        return img
    
    def export_edited_map(self, output_path):
        """Export edited map as new PGM file"""
        try:
            rendered = self.render_map_with_layers()
            if rendered is None:
                return False
            
            # Convert back to grayscale
            gray_map = rendered.convert('L')
            
            # Save as PGM
            gray_map.save(output_path)
            
            # Create corresponding YAML
            yaml_path = output_path.replace('.pgm', '.yaml')
            yaml_data = {
                'image': output_path,
                'resolution': self.map_info.get('resolution', 0.05),
                'origin': self.map_info.get('origin', [0, 0, 0]),
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            
            with open(yaml_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            
            self.get_logger().info(f'Exported edited map to {output_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to export map: {e}')
            return False


# Create Flask app
app = Flask(__name__,
            template_folder='/home/aun/Downloads/smr300l_gazebo_ros2control-main/src/map_editor/map_editor/web/templates')
CORS(app)

# Global node reference
editor_node = None


@app.route('/')
def index():
    return render_template('map_editor.html')


@app.route('/api/map/info')
def get_map_info():
    """Get current map information"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    return jsonify({
        'resolution': editor_node.map_info.get('resolution'),
        'origin': editor_node.map_info.get('origin'),
        'width': editor_node.original_map.width if editor_node.original_map else 0,
        'height': editor_node.original_map.height if editor_node.original_map else 0
    })


@app.route('/api/map/preview')
def get_map_preview():
    """Get rendered map with layers"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    try:
        rendered = editor_node.render_map_with_layers()
        if rendered is None:
            return jsonify({'error': 'No map loaded'}), 500
        
        # Convert to base64
        img_io = io.BytesIO()
        rendered.save(img_io, 'PNG')
        img_io.seek(0)
        img_base64 = base64.b64encode(img_io.getvalue()).decode('utf-8')
        
        return jsonify({
            'image': img_base64,
            'width': rendered.width,
            'height': rendered.height
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/layers', methods=['GET'])
def get_layers():
    """Get all map layers"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    return jsonify(editor_node.layers)


@app.route('/api/layer/add', methods=['POST'])
def add_layer_object():
    """Add object to a layer"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.json
    layer_type = data.get('layer')
    obj = data.get('object')
    
    if not layer_type or not obj:
        return jsonify({'error': 'Missing layer or object'}), 400
    
    success = editor_node.add_layer_object(layer_type, obj)
    return jsonify({'ok': success})


@app.route('/api/layer/clear', methods=['POST'])
def clear_layer():
    """Clear all objects from a layer"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.json
    layer_type = data.get('layer')
    
    if not layer_type:
        return jsonify({'error': 'Missing layer type'}), 400
    
    success = editor_node.clear_layer(layer_type)
    return jsonify({'ok': success})


@app.route('/api/map/export', methods=['POST'])
def export_map():
    """Export edited map to new PGM file"""
    if editor_node is None:
        return jsonify({'error': 'Node not initialized'}), 500
    
    data = request.json
    output_path = data.get('path', '/tmp/edited_map.pgm')
    
    success = editor_node.export_edited_map(output_path)
    return jsonify({'ok': success, 'path': output_path})


def run_flask():
    """Run Flask server"""
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)


def main(args=None):
    global editor_node
    
    rclpy.init(args=args)
    editor_node = MapEditorServer()
    
    # Start Flask in separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    editor_node.get_logger().info('Map Editor UI available at: http://localhost:5001')
    
    try:
        rclpy.spin(editor_node)
    except KeyboardInterrupt:
        pass
    finally:
        editor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

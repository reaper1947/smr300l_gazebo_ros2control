// Professional Map Editor - Interactive Canvas

const API_BASE = 'http://localhost:5001/api';

// State
let mapInfo = null;
let mapImage = null;
let currentTool = null;
let drawingPoints = [];
let tempDrawing = null;
let layers = {
    obstacles: [],
    no_go_zones: [],
    slow_zones: [],
    restricted: []
};
let visibleLayers = {
    obstacles: true,
    no_go_zones: true,
    slow_zones: true,
    restricted: true
};

// Canvas setup
const canvas = document.getElementById('map-canvas');
const ctx = canvas.getContext('2d');

// Initialize
async function init() {
    await loadMapInfo();
    await loadLayers();
    await loadMapPreview();
    setupEventListeners();
    updateStatus('Ready to edit');
}

// Load map information
async function loadMapInfo() {
    try {
        const response = await fetch(`${API_BASE}/map/info`);
        mapInfo = await response.json();
        
        document.getElementById('resolution-info').textContent = 
            `Resolution: ${mapInfo.resolution}m/px`;
        document.getElementById('map-size').textContent = 
            `Map: ${mapInfo.width} x ${mapInfo.height}`;
    } catch (error) {
        console.error('Failed to load map info:', error);
        updateStatus('Error loading map info');
    }
}

// Load map layers
async function loadLayers() {
    try {
        const response = await fetch(`${API_BASE}/layers`);
        layers = await response.json();
        updateObjectCount();
    } catch (error) {
        console.error('Failed to load layers:', error);
    }
}

// Load map preview with layers
async function loadMapPreview() {
    try {
        const response = await fetch(`${API_BASE}/map/preview`);
        const data = await response.json();
        
        const img = new Image();
        img.onload = () => {
            mapImage = img;
            canvas.width = img.width;
            canvas.height = img.height;
            redrawCanvas();
        };
        img.src = `data:image/png;base64,${data.image}`;
    } catch (error) {
        console.error('Failed to load map preview:', error);
        updateStatus('Error loading map');
    }
}

// Redraw canvas
function redrawCanvas() {
    if (!mapImage) return;
    
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(mapImage, 0, 0);
    
    // Draw temporary drawing
    if (tempDrawing) {
        drawTempObject(tempDrawing);
    }
}

// Draw temporary object during creation
function drawTempObject(obj) {
    ctx.save();
    
    if (obj.type === 'rectangle') {
        ctx.strokeStyle = obj.color || '#ff0000';
        ctx.lineWidth = 3;
        ctx.setLineDash([5, 5]);
        const width = obj.x2 - obj.x1;
        const height = obj.y2 - obj.y1;
        ctx.strokeRect(obj.x1, obj.y1, width, height);
    } else if (obj.type === 'circle') {
        ctx.strokeStyle = obj.color || '#ff0000';
        ctx.lineWidth = 3;
        ctx.setLineDash([5, 5]);
        const radius = Math.sqrt(
            Math.pow(obj.x2 - obj.x1, 2) + Math.pow(obj.y2 - obj.y1, 2)
        );
        ctx.beginPath();
        ctx.arc(obj.x1, obj.y1, radius, 0, 2 * Math.PI);
        ctx.stroke();
    } else if (obj.type === 'polygon' && obj.points && obj.points.length > 0) {
        ctx.strokeStyle = obj.color || '#ff0000';
        ctx.fillStyle = obj.color ? obj.color + '40' : 'rgba(255, 0, 0, 0.2)';
        ctx.lineWidth = 3;
        ctx.setLineDash([5, 5]);
        
        ctx.beginPath();
        ctx.moveTo(obj.points[0].x, obj.points[0].y);
        for (let i = 1; i < obj.points.length; i++) {
            ctx.lineTo(obj.points[i].x, obj.points[i].y);
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        
        // Draw points
        obj.points.forEach(p => {
            ctx.fillStyle = '#ff0000';
            ctx.fillRect(p.x - 3, p.y - 3, 6, 6);
        });
    }
    
    ctx.restore();
}

// Convert pixel coordinates to world coordinates
function pixelToWorld(px, py) {
    const wx = mapInfo.origin[0] + px * mapInfo.resolution;
    const wy = mapInfo.origin[1] + (mapInfo.height - py) * mapInfo.resolution;
    return { x: wx, y: wy };
}

// Convert world coordinates to pixel coordinates
function worldToPixel(wx, wy) {
    const px = (wx - mapInfo.origin[0]) / mapInfo.resolution;
    const py = mapInfo.height - (wy - mapInfo.origin[1]) / mapInfo.resolution;
    return { x: px, y: py };
}

// Calculate distance between two points
function distance(x1, y1, x2, y2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
}

// Calculate polygon area
function polygonArea(points) {
    let area = 0;
    for (let i = 0; i < points.length; i++) {
        const j = (i + 1) % points.length;
        area += points[i].x * points[j].y;
        area -= points[j].x * points[i].y;
    }
    return Math.abs(area / 2.0);
}

// Setup event listeners
function setupEventListeners() {
    // Tool buttons
    document.querySelectorAll('.tool-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            currentTool = btn.dataset.tool;
            drawingPoints = [];
            tempDrawing = null;
            updateInstructions();
        });
    });
    
    // Layer visibility toggles
    document.getElementById('layer-obstacles').addEventListener('change', (e) => {
        visibleLayers.obstacles = e.target.checked;
        loadMapPreview();
    });
    document.getElementById('layer-nogo').addEventListener('change', (e) => {
        visibleLayers.no_go_zones = e.target.checked;
        loadMapPreview();
    });
    document.getElementById('layer-slow').addEventListener('change', (e) => {
        visibleLayers.slow_zones = e.target.checked;
        loadMapPreview();
    });
    document.getElementById('layer-restricted').addEventListener('change', (e) => {
        visibleLayers.restricted = e.target.checked;
        loadMapPreview();
    });
    
    // Clear buttons
    document.getElementById('btn-clear-obstacles').addEventListener('click', () => clearLayer('obstacles'));
    document.getElementById('btn-clear-nogo').addEventListener('click', () => clearLayer('no_go_zones'));
    document.getElementById('btn-clear-slow').addEventListener('click', () => clearLayer('slow_zones'));
    document.getElementById('btn-clear-all').addEventListener('click', clearAllLayers);
    document.getElementById('btn-export').addEventListener('click', exportMap);
    
    // Canvas mouse events
    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('click', handleClick);
    canvas.addEventListener('contextmenu', handleRightClick);
}

// Handle mouse move
function handleMouseMove(e) {
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const world = pixelToWorld(px, py);
    
    document.getElementById('coords-display').textContent = 
        `X: ${world.x.toFixed(2)}m Y: ${world.y.toFixed(2)}m`;
    
    // Update temporary drawing
    if (drawingPoints.length > 0) {
        if (currentTool.includes('rect') || currentTool.includes('nogo') || 
            currentTool.includes('slow') || currentTool.includes('restricted')) {
            tempDrawing = {
                type: 'rectangle',
                x1: drawingPoints[0].px,
                y1: drawingPoints[0].py,
                x2: px,
                y2: py,
                color: getToolColor()
            };
        } else if (currentTool.includes('circle')) {
            tempDrawing = {
                type: 'circle',
                x1: drawingPoints[0].px,
                y1: drawingPoints[0].py,
                x2: px,
                y2: py,
                color: getToolColor()
            };
        } else if (currentTool.includes('poly')) {
            tempDrawing = {
                type: 'polygon',
                points: [...drawingPoints.map(p => ({ x: p.px, y: p.py })), { x: px, y: py }],
                color: getToolColor()
            };
        }
        redrawCanvas();
    }
}

// Handle click
function handleClick(e) {
    if (!currentTool) {
        updateStatus('Please select a tool first');
        return;
    }
    
    const rect = canvas.getBoundingClientRect();
    const px = e.clientX - rect.left;
    const py = e.clientY - rect.top;
    const world = pixelToWorld(px, py);
    
    if (currentTool === 'measure') {
        handleMeasure(px, py, world);
    } else if (currentTool === 'area') {
        handleAreaMeasure(px, py, world);
    } else if (currentTool.includes('poly')) {
        handlePolygonDraw(px, py, world);
    } else {
        handleShapeDraw(px, py, world);
    }
}

// Handle right click (finish polygon)
function handleRightClick(e) {
    e.preventDefault();
    
    if (currentTool && currentTool.includes('poly') && drawingPoints.length >= 3) {
        finishPolygon();
    }
}

// Handle shape drawing (rectangle, circle)
function handleShapeDraw(px, py, world) {
    if (drawingPoints.length === 0) {
        drawingPoints.push({ px, py, world });
        updateStatus('Click again to finish shape');
    } else {
        const start = drawingPoints[0];
        const layer = getLayerForTool();
        
        let obj = {};
        if (currentTool.includes('rect') || currentTool.includes('nogo') || 
            currentTool.includes('slow') || currentTool.includes('restricted')) {
            obj = {
                type: 'rectangle',
                x1: Math.min(start.world.x, world.x),
                y1: Math.min(start.world.y, world.y),
                x2: Math.max(start.world.x, world.x),
                y2: Math.max(start.world.y, world.y)
            };
            
            if (currentTool === 'slow') {
                obj.speed_limit = parseFloat(document.getElementById('speed-limit').value);
            }
        } else if (currentTool.includes('circle')) {
            obj = {
                type: 'circle',
                x: start.world.x,
                y: start.world.y,
                radius: distance(start.world.x, start.world.y, world.x, world.y)
            };
        }
        
        addLayerObject(layer, obj);
        drawingPoints = [];
        tempDrawing = null;
        updateStatus('Shape added');
    }
}

// Handle polygon drawing
function handlePolygonDraw(px, py, world) {
    drawingPoints.push({ px, py, world });
    updateStatus(`Polygon: ${drawingPoints.length} points (right-click to finish)`);
}

// Finish polygon
async function finishPolygon() {
    if (drawingPoints.length < 3) return;
    
    const layer = getLayerForTool();
    const obj = {
        type: 'polygon',
        points: drawingPoints.map(p => ({ x: p.world.x, y: p.world.y }))
    };
    
    await addLayerObject(layer, obj);
    drawingPoints = [];
    tempDrawing = null;
    updateStatus('Polygon added');
}

// Handle distance measurement
function handleMeasure(px, py, world) {
    if (drawingPoints.length === 0) {
        drawingPoints.push({ px, py, world });
        updateStatus('Click second point');
    } else {
        const start = drawingPoints[0];
        const dist = distance(start.world.x, start.world.y, world.x, world.y);
        document.getElementById('measurement-result').textContent = 
            `Distance: ${dist.toFixed(3)}m`;
        drawingPoints = [];
        updateStatus('Measurement complete');
    }
}

// Handle area measurement
function handleAreaMeasure(px, py, world) {
    drawingPoints.push({ px, py, world });
    
    if (drawingPoints.length < 3) {
        updateStatus(`Area: ${drawingPoints.length} points (right-click to finish)`);
    }
    
    // Calculate area if closed (right-click handled separately)
}

// Get layer name for current tool
function getLayerForTool() {
    if (currentTool.includes('obstacle')) return 'obstacles';
    if (currentTool.includes('nogo')) return 'no_go_zones';
    if (currentTool.includes('slow')) return 'slow_zones';
    if (currentTool.includes('restricted')) return 'restricted';
    return 'obstacles';
}

// Get color for current tool
function getToolColor() {
    if (currentTool.includes('obstacle')) return '#000000';
    if (currentTool.includes('nogo')) return '#ff0000';
    if (currentTool.includes('slow')) return '#ffff00';
    if (currentTool.includes('restricted')) return '#ffa500';
    return '#000000';
}

// Add object to layer
async function addLayerObject(layer, obj) {
    try {
        const response = await fetch(`${API_BASE}/layer/add`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ layer, object: obj })
        });
        
        if (response.ok) {
            await loadLayers();
            await loadMapPreview();
            updateObjectCount();
        }
    } catch (error) {
        console.error('Failed to add object:', error);
        updateStatus('Error adding object');
    }
}

// Clear layer
async function clearLayer(layer) {
    if (!confirm(`Clear all ${layer.replace('_', ' ')}?`)) return;
    
    try {
        const response = await fetch(`${API_BASE}/layer/clear`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ layer })
        });
        
        if (response.ok) {
            await loadLayers();
            await loadMapPreview();
            updateObjectCount();
            updateStatus(`Cleared ${layer}`);
        }
    } catch (error) {
        console.error('Failed to clear layer:', error);
    }
}

// Clear all layers
async function clearAllLayers() {
    if (!confirm('Clear ALL layers? This cannot be undone!')) return;
    
    for (const layer of Object.keys(layers)) {
        await clearLayer(layer);
    }
}

// Export map
async function exportMap() {
    const defaultName = 'edited_map_' + new Date().toISOString().slice(0,10).replace(/-/g, '');
    const baseName = prompt('Enter map name (without extension):', defaultName);
    if (!baseName) return;

    const pgmFilename = baseName.toLowerCase().endsWith('.pgm') ? baseName : `${baseName}.pgm`;
    
    try {
        updateStatus('Exporting map...');
        const response = await fetch(`${API_BASE}/map/export`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ path: pgmFilename })
        });
        
        if (response.ok) {
            const result = await response.json();
            const downloadPgm = result.download || `/api/map/download/${pgmFilename}`;
            const downloadYaml = downloadPgm.replace(/\.pgm$/, '.yaml');

            // Download PGM
            const aPgm = document.createElement('a');
            aPgm.href = downloadPgm;
            aPgm.download = pgmFilename;
            document.body.appendChild(aPgm);
            aPgm.click();
            document.body.removeChild(aPgm);

            // Download YAML after a short delay
            setTimeout(() => {
                const aYaml = document.createElement('a');
                aYaml.href = downloadYaml;
                aYaml.download = pgmFilename.replace(/\.pgm$/, '.yaml');
                document.body.appendChild(aYaml);
                aYaml.click();
                document.body.removeChild(aYaml);
            }, 500);

            updateStatus(`Map exported to ${downloadPgm}`);
            alert(`Map successfully exported:\n${pgmFilename}\n${pgmFilename.replace(/\\.pgm$/, '.yaml')}`);
        } else {
            updateStatus('Export failed');
        }
    } catch (error) {
        console.error('Failed to export:', error);
        updateStatus('Export error');
    }
}

// Update instructions
function updateInstructions() {
    const instructions = document.getElementById('instructions');
    
    if (!currentTool) {
        instructions.textContent = 'Select a tool to begin editing';
        return;
    }
    
    const messages = {
        'obstacle-rect': 'Click two points to draw obstacle rectangle',
        'obstacle-circle': 'Click center, then click radius point',
        'obstacle-poly': 'Click to add points, right-click to finish polygon',
        'nogo': 'Click two corners to draw no-go zone',
        'slow': 'Click two corners to draw slow zone',
        'restricted': 'Click two corners to draw restricted area',
        'measure': 'Click two points to measure distance',
        'area': 'Click points to form area, right-click to finish'
    };
    
    instructions.textContent = messages[currentTool] || 'Draw on the map';
}

// Update status
function updateStatus(text) {
    document.getElementById('status-text').textContent = text;
}

// Update object count
function updateObjectCount() {
    const total = Object.values(layers).reduce((sum, arr) => sum + arr.length, 0);
    document.getElementById('object-count').textContent = `Objects: ${total}`;
}

// Initialize on page load
init();

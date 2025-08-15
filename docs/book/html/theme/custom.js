// Tung Playground Documentation Custom JavaScript

document.addEventListener('DOMContentLoaded', function() {
    // Add copy button to code blocks
    addCopyButtons();
    
    // Add progress indicators
    addProgressIndicators();
    
    // Add interactive elements
    addInteractiveElements();
    
    // Add performance metrics
    addPerformanceMetrics();
    
    // Initialize theme switcher
    initializeThemeSwitcher();
});

function addCopyButtons() {
    const codeBlocks = document.querySelectorAll('pre code');
    
    codeBlocks.forEach(function(codeBlock) {
        const pre = codeBlock.parentNode;
        const button = document.createElement('button');
        
        button.className = 'copy-button';
        button.textContent = '📋 Copy';
        button.style.cssText = `
            position: absolute;
            top: 0.5rem;
            right: 0.5rem;
            background: var(--tung-primary);
            color: white;
            border: none;
            padding: 0.25rem 0.5rem;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.75rem;
            opacity: 0;
            transition: opacity 0.2s ease;
        `;
        
        pre.style.position = 'relative';
        pre.appendChild(button);
        
        pre.addEventListener('mouseenter', function() {
            button.style.opacity = '1';
        });
        
        pre.addEventListener('mouseleave', function() {
            button.style.opacity = '0';
        });
        
        button.addEventListener('click', function() {
            navigator.clipboard.writeText(codeBlock.textContent).then(function() {
                button.textContent = '✅ Copied!';
                setTimeout(function() {
                    button.textContent = '📋 Copy';
                }, 2000);
            });
        });
    });
}

function addProgressIndicators() {
    // Add reading progress indicator
    const progressBar = document.createElement('div');
    progressBar.className = 'reading-progress';
    progressBar.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 0%;
        height: 4px;
        background: linear-gradient(90deg, var(--tung-primary), var(--tung-accent));
        z-index: 9999;
        transition: width 0.1s ease;
    `;
    
    document.body.appendChild(progressBar);
    
    window.addEventListener('scroll', function() {
        const scrollTop = window.pageYOffset;
        const docHeight = document.body.scrollHeight - window.innerHeight;
        const scrollPercent = (scrollTop / docHeight) * 100;
        progressBar.style.width = scrollPercent + '%';
    });
}

function addInteractiveElements() {
    // Add interactive pipeline diagram
    const pipelineDiagrams = document.querySelectorAll('.pipeline-diagram');
    
    pipelineDiagrams.forEach(function(diagram) {
        const stages = ['Image', '3D Gen', 'Decomp', 'Rigging', 'URDF', 'Sim', 'Training'];
        const container = document.createElement('div');
        container.className = 'interactive-pipeline';
        container.style.cssText = `
            display: flex;
            gap: 1rem;
            margin: 2rem 0;
            padding: 1rem;
            background: linear-gradient(135deg, #f8f9fa, #e3f2fd);
            border-radius: 8px;
            overflow-x: auto;
        `;
        
        stages.forEach(function(stage, index) {
            const stageElement = document.createElement('div');
            stageElement.className = 'pipeline-stage';
            stageElement.textContent = stage;
            stageElement.style.cssText = `
                flex: 1;
                min-width: 80px;
                padding: 1rem;
                background: white;
                border: 2px solid var(--tung-accent);
                border-radius: 8px;
                text-align: center;
                font-weight: 600;
                cursor: pointer;
                transition: all 0.3s ease;
                position: relative;
            `;
            
            if (index < stages.length - 1) {
                const arrow = document.createElement('div');
                arrow.textContent = '→';
                arrow.style.cssText = `
                    position: absolute;
                    right: -1.5rem;
                    top: 50%;
                    transform: translateY(-50%);
                    font-size: 1.5rem;
                    color: var(--tung-primary);
                `;
                stageElement.appendChild(arrow);
            }
            
            stageElement.addEventListener('click', function() {
                stageElement.style.background = 'var(--tung-primary)';
                stageElement.style.color = 'white';
                stageElement.style.transform = 'scale(1.05)';
                
                setTimeout(function() {
                    stageElement.style.background = 'white';
                    stageElement.style.color = 'inherit';
                    stageElement.style.transform = 'scale(1)';
                }, 500);
            });
            
            container.appendChild(stageElement);
        });
        
        diagram.replaceWith(container);
    });
}

function addPerformanceMetrics() {
    // Add performance timing display
    const performanceSection = document.querySelector('.performance-metrics');
    
    if (performanceSection) {
        const metrics = {
            'Generation': { time: '1.02s', percentage: 18 },
            'Decomposition': { time: '0.83s', percentage: 15 },
            'Rigging': { time: '0.61s', percentage: 11 },
            'URDF': { time: '0.52s', percentage: 9 },
            'Simulation': { time: '0.43s', percentage: 8 },
            'Training': { time: '2.15s', percentage: 39 }
        };
        
        const chartContainer = document.createElement('div');
        chartContainer.className = 'performance-chart';
        chartContainer.style.cssText = `
            margin: 2rem 0;
            padding: 1rem;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        `;
        
        Object.entries(metrics).forEach(function([stage, data]) {
            const row = document.createElement('div');
            row.style.cssText = `
                display: flex;
                align-items: center;
                margin: 0.5rem 0;
                gap: 1rem;
            `;
            
            const label = document.createElement('div');
            label.textContent = stage;
            label.style.cssText = `
                width: 120px;
                font-weight: 600;
            `;
            
            const barContainer = document.createElement('div');
            barContainer.style.cssText = `
                flex: 1;
                height: 20px;
                background: #e2e8f0;
                border-radius: 10px;
                overflow: hidden;
                position: relative;
            `;
            
            const bar = document.createElement('div');
            bar.style.cssText = `
                height: 100%;
                width: ${data.percentage}%;
                background: linear-gradient(90deg, var(--tung-primary), var(--tung-accent));
                border-radius: 10px;
                transition: width 1s ease;
                animation: fillBar 1s ease-in-out;
            `;
            
            const timeLabel = document.createElement('div');
            timeLabel.textContent = data.time;
            timeLabel.style.cssText = `
                width: 60px;
                text-align: right;
                font-family: monospace;
                color: var(--tung-secondary);
            `;
            
            barContainer.appendChild(bar);
            row.appendChild(label);
            row.appendChild(barContainer);
            row.appendChild(timeLabel);
            chartContainer.appendChild(row);
        });
        
        performanceSection.appendChild(chartContainer);
    }
}

function initializeThemeSwitcher() {
    // Add theme switcher button
    const themeButton = document.createElement('button');
    themeButton.innerHTML = '🌙';
    themeButton.className = 'theme-switcher';
    themeButton.style.cssText = `
        position: fixed;
        bottom: 2rem;
        right: 2rem;
        width: 50px;
        height: 50px;
        border-radius: 50%;
        border: none;
        background: var(--tung-primary);
        color: white;
        font-size: 1.5rem;
        cursor: pointer;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        transition: all 0.3s ease;
        z-index: 1000;
    `;
    
    themeButton.addEventListener('click', function() {
        document.body.classList.toggle('dark-theme');
        themeButton.innerHTML = document.body.classList.contains('dark-theme') ? '☀️' : '🌙';
        
        // Save preference
        localStorage.setItem('tung-docs-theme', 
            document.body.classList.contains('dark-theme') ? 'dark' : 'light'
        );
    });
    
    themeButton.addEventListener('mouseenter', function() {
        themeButton.style.transform = 'scale(1.1) rotate(15deg)';
    });
    
    themeButton.addEventListener('mouseleave', function() {
        themeButton.style.transform = 'scale(1) rotate(0deg)';
    });
    
    // Load saved theme
    const savedTheme = localStorage.getItem('tung-docs-theme');
    if (savedTheme === 'dark') {
        document.body.classList.add('dark-theme');
        themeButton.innerHTML = '☀️';
    }
    
    document.body.appendChild(themeButton);
}

// Add CSS animations
const style = document.createElement('style');
style.textContent = `
    @keyframes fillBar {
        from { width: 0%; }
        to { width: var(--final-width); }
    }
    
    .dark-theme {
        --bg-color: #1a1a1a;
        --text-color: #e2e8f0;
        --link-color: #60a5fa;
    }
    
    .dark-theme .content {
        background-color: var(--bg-color);
        color: var(--text-color);
    }
    
    .dark-theme .nav-chapters {
        background: linear-gradient(135deg, #2d3748, #1a202c);
    }
    
    .dark-theme table {
        background-color: #2d3748;
        color: var(--text-color);
    }
    
    .dark-theme th {
        background: linear-gradient(135deg, #4a5568, #2d3748);
    }
    
    .dark-theme tr:nth-child(even) {
        background-color: #374151;
    }
    
    .dark-theme tr:hover {
        background-color: #4a5568;
    }
    
    .pulse {
        animation: pulse 2s infinite;
    }
    
    @keyframes pulse {
        0% { opacity: 1; }
        50% { opacity: 0.5; }
        100% { opacity: 1; }
    }
`;
document.head.appendChild(style);

// Add search enhancements
function enhanceSearch() {
    const searchInput = document.querySelector('#searchbar');
    if (searchInput) {
        // Add search suggestions
        const suggestionsDiv = document.createElement('div');
        suggestionsDiv.className = 'search-suggestions';
        suggestionsDiv.style.cssText = `
            position: absolute;
            top: 100%;
            left: 0;
            right: 0;
            background: white;
            border: 1px solid #e2e8f0;
            border-top: none;
            border-radius: 0 0 8px 8px;
            max-height: 200px;
            overflow-y: auto;
            z-index: 1000;
            display: none;
        `;
        
        searchInput.parentNode.style.position = 'relative';
        searchInput.parentNode.appendChild(suggestionsDiv);
        
        // Add popular search terms
        const popularTerms = [
            'pipeline', 'hero', 'generation', 'decomposition', 
            'rigging', 'urdf', 'simulation', 'training',
            'configuration', 'plugins', 'api', 'examples'
        ];
        
        searchInput.addEventListener('focus', function() {
            if (searchInput.value === '') {
                suggestionsDiv.innerHTML = '<div style="padding: 0.5rem; font-weight: 600; color: var(--tung-secondary);">Popular searches:</div>';
                popularTerms.forEach(term => {
                    const suggestion = document.createElement('div');
                    suggestion.textContent = term;
                    suggestion.style.cssText = `
                        padding: 0.5rem;
                        cursor: pointer;
                        transition: background-color 0.2s ease;
                    `;
                    suggestion.addEventListener('mouseenter', function() {
                        suggestion.style.backgroundColor = '#f8f9fa';
                    });
                    suggestion.addEventListener('mouseleave', function() {
                        suggestion.style.backgroundColor = 'transparent';
                    });
                    suggestion.addEventListener('click', function() {
                        searchInput.value = term;
                        suggestionsDiv.style.display = 'none';
                        // Trigger search
                        const event = new Event('input');
                        searchInput.dispatchEvent(event);
                    });
                    suggestionsDiv.appendChild(suggestion);
                });
                suggestionsDiv.style.display = 'block';
            }
        });
        
        document.addEventListener('click', function(event) {
            if (!searchInput.contains(event.target) && !suggestionsDiv.contains(event.target)) {
                suggestionsDiv.style.display = 'none';
            }
        });
    }
}

// Initialize search enhancements after a short delay
setTimeout(enhanceSearch, 500);
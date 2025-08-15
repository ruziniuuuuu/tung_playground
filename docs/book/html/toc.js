// Populate the sidebar
//
// This is a script, and not included directly in the page, to control the total size of the book.
// The TOC contains an entry for each page, so if each page includes a copy of the TOC,
// the total size of the page becomes O(n**2).
class MDBookSidebarScrollbox extends HTMLElement {
    constructor() {
        super();
    }
    connectedCallback() {
        this.innerHTML = '<ol class="chapter"><li class="chapter-item expanded affix "><a href="introduction.html">Introduction</a></li><li class="chapter-item expanded affix "><li class="part-title">Getting Started</li><li class="chapter-item expanded "><a href="getting-started/quick-start.html"><strong aria-hidden="true">1.</strong> Quick Start</a></li><li class="chapter-item expanded "><a href="getting-started/installation.html"><strong aria-hidden="true">2.</strong> Installation</a></li><li class="chapter-item expanded "><a href="getting-started/basic-usage.html"><strong aria-hidden="true">3.</strong> Basic Usage</a></li><li class="chapter-item expanded "><a href="getting-started/first-hero.html"><strong aria-hidden="true">4.</strong> Your First Hero</a></li><li class="chapter-item expanded affix "><li class="part-title">Architecture</li><li class="chapter-item expanded "><a href="architecture/overview.html"><strong aria-hidden="true">5.</strong> Overview</a></li><li class="chapter-item expanded "><a href="architecture/core-components.html"><strong aria-hidden="true">6.</strong> Core Components</a></li><li class="chapter-item expanded "><a href="architecture/pipeline.html"><strong aria-hidden="true">7.</strong> Pipeline System</a></li><li class="chapter-item expanded "><a href="architecture/plugins.html"><strong aria-hidden="true">8.</strong> Plugin Architecture</a></li><li class="chapter-item expanded "><a href="architecture/configuration.html"><strong aria-hidden="true">9.</strong> Configuration System</a></li><li class="chapter-item expanded affix "><li class="part-title">Pipeline Stages</li><li class="chapter-item expanded "><a href="pipeline/generation.html"><strong aria-hidden="true">10.</strong> Image-to-3D Generation</a></li><li class="chapter-item expanded "><a href="pipeline/decomposition.html"><strong aria-hidden="true">11.</strong> Part Decomposition</a></li><li class="chapter-item expanded "><a href="pipeline/rigging.html"><strong aria-hidden="true">12.</strong> Skeleton Rigging</a></li><li class="chapter-item expanded "><a href="pipeline/urdf.html"><strong aria-hidden="true">13.</strong> URDF Generation</a></li><li class="chapter-item expanded "><a href="pipeline/simulation.html"><strong aria-hidden="true">14.</strong> Simulation Setup</a></li><li class="chapter-item expanded "><a href="pipeline/training.html"><strong aria-hidden="true">15.</strong> RL Training</a></li><li class="chapter-item expanded affix "><li class="part-title">Development Guide</li><li class="chapter-item expanded "><a href="development/setup.html"><strong aria-hidden="true">16.</strong> Development Setup</a></li><li class="chapter-item expanded "><a href="development/plugins.html"><strong aria-hidden="true">17.</strong> Creating Plugins</a></li><li class="chapter-item expanded "><a href="development/testing.html"><strong aria-hidden="true">18.</strong> Testing</a></li><li class="chapter-item expanded "><a href="development/contributing.html"><strong aria-hidden="true">19.</strong> Contributing</a></li><li class="chapter-item expanded "><a href="development/code-style.html"><strong aria-hidden="true">20.</strong> Code Style</a></li><li class="chapter-item expanded affix "><li class="part-title">API Reference</li><li class="chapter-item expanded "><a href="api/core.html"><strong aria-hidden="true">21.</strong> Core API</a></li><li class="chapter-item expanded "><a href="api/hero.html"><strong aria-hidden="true">22.</strong> Hero Management</a></li><li class="chapter-item expanded "><a href="api/pipeline.html"><strong aria-hidden="true">23.</strong> Pipeline API</a></li><li class="chapter-item expanded "><a href="api/plugin.html"><strong aria-hidden="true">24.</strong> Plugin API</a></li><li class="chapter-item expanded "><a href="api/utilities.html"><strong aria-hidden="true">25.</strong> Utilities</a></li><li class="chapter-item expanded affix "><li class="part-title">Integration</li><li class="chapter-item expanded "><a href="integration/mujoco.html"><strong aria-hidden="true">26.</strong> MuJoCo Integration</a></li><li class="chapter-item expanded "><a href="integration/isaac-lab.html"><strong aria-hidden="true">27.</strong> Isaac Lab Integration</a></li><li class="chapter-item expanded "><a href="integration/wonder3d.html"><strong aria-hidden="true">28.</strong> Wonder3D Integration</a></li><li class="chapter-item expanded "><a href="integration/partcrafter.html"><strong aria-hidden="true">29.</strong> PartCrafter Integration</a></li><li class="chapter-item expanded affix "><li class="part-title">Examples</li><li class="chapter-item expanded "><a href="examples/complete-demo.html"><strong aria-hidden="true">30.</strong> Complete Pipeline Demo</a></li><li class="chapter-item expanded "><a href="examples/custom-pipeline.html"><strong aria-hidden="true">31.</strong> Custom Pipeline</a></li><li class="chapter-item expanded "><a href="examples/batch-processing.html"><strong aria-hidden="true">32.</strong> Batch Processing</a></li><li class="chapter-item expanded "><a href="examples/advanced-config.html"><strong aria-hidden="true">33.</strong> Advanced Configuration</a></li><li class="chapter-item expanded affix "><li class="part-title">Deployment</li><li class="chapter-item expanded "><a href="deployment/production.html"><strong aria-hidden="true">34.</strong> Production Deployment</a></li><li class="chapter-item expanded "><a href="deployment/docker.html"><strong aria-hidden="true">35.</strong> Docker Setup</a></li><li class="chapter-item expanded "><a href="deployment/performance.html"><strong aria-hidden="true">36.</strong> Performance Tuning</a></li><li class="chapter-item expanded "><a href="deployment/monitoring.html"><strong aria-hidden="true">37.</strong> Monitoring</a></li><li class="chapter-item expanded affix "><li class="part-title">Troubleshooting</li><li class="chapter-item expanded "><a href="troubleshooting/common-issues.html"><strong aria-hidden="true">38.</strong> Common Issues</a></li><li class="chapter-item expanded "><a href="troubleshooting/performance.html"><strong aria-hidden="true">39.</strong> Performance Problems</a></li><li class="chapter-item expanded "><a href="troubleshooting/debugging.html"><strong aria-hidden="true">40.</strong> Debug Guide</a></li><li class="chapter-item expanded "><a href="troubleshooting/faq.html"><strong aria-hidden="true">41.</strong> FAQ</a></li><li class="chapter-item expanded affix "><li class="part-title">Appendices</li><li class="chapter-item expanded "><a href="appendices/config-reference.html"><strong aria-hidden="true">42.</strong> Configuration Reference</a></li><li class="chapter-item expanded "><a href="appendices/file-formats.html"><strong aria-hidden="true">43.</strong> File Formats</a></li><li class="chapter-item expanded "><a href="appendices/glossary.html"><strong aria-hidden="true">44.</strong> Glossary</a></li><li class="chapter-item expanded "><a href="appendices/changelog.html"><strong aria-hidden="true">45.</strong> Changelog</a></li></ol>';
        // Set the current, active page, and reveal it if it's hidden
        let current_page = document.location.href.toString().split("#")[0].split("?")[0];
        if (current_page.endsWith("/")) {
            current_page += "index.html";
        }
        var links = Array.prototype.slice.call(this.querySelectorAll("a"));
        var l = links.length;
        for (var i = 0; i < l; ++i) {
            var link = links[i];
            var href = link.getAttribute("href");
            if (href && !href.startsWith("#") && !/^(?:[a-z+]+:)?\/\//.test(href)) {
                link.href = path_to_root + href;
            }
            // The "index" page is supposed to alias the first chapter in the book.
            if (link.href === current_page || (i === 0 && path_to_root === "" && current_page.endsWith("/index.html"))) {
                link.classList.add("active");
                var parent = link.parentElement;
                if (parent && parent.classList.contains("chapter-item")) {
                    parent.classList.add("expanded");
                }
                while (parent) {
                    if (parent.tagName === "LI" && parent.previousElementSibling) {
                        if (parent.previousElementSibling.classList.contains("chapter-item")) {
                            parent.previousElementSibling.classList.add("expanded");
                        }
                    }
                    parent = parent.parentElement;
                }
            }
        }
        // Track and set sidebar scroll position
        this.addEventListener('click', function(e) {
            if (e.target.tagName === 'A') {
                sessionStorage.setItem('sidebar-scroll', this.scrollTop);
            }
        }, { passive: true });
        var sidebarScrollTop = sessionStorage.getItem('sidebar-scroll');
        sessionStorage.removeItem('sidebar-scroll');
        if (sidebarScrollTop) {
            // preserve sidebar scroll position when navigating via links within sidebar
            this.scrollTop = sidebarScrollTop;
        } else {
            // scroll sidebar to current active section when navigating via "next/previous chapter" buttons
            var activeSection = document.querySelector('#sidebar .active');
            if (activeSection) {
                activeSection.scrollIntoView({ block: 'center' });
            }
        }
        // Toggle buttons
        var sidebarAnchorToggles = document.querySelectorAll('#sidebar a.toggle');
        function toggleSection(ev) {
            ev.currentTarget.parentElement.classList.toggle('expanded');
        }
        Array.from(sidebarAnchorToggles).forEach(function (el) {
            el.addEventListener('click', toggleSection);
        });
    }
}
window.customElements.define("mdbook-sidebar-scrollbox", MDBookSidebarScrollbox);

#!/usr/bin/env python3
# encoding: utf-8
"""Use instead of `python3 -m http.server` when you need CORS"""

from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
import sys
import os


class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        # Comprehensive CORS headers
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, HEAD, PUT, DELETE')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.send_header('Access-Control-Allow-Credentials', 'true')
        self.send_header('Access-Control-Max-Age', '86400')
        self.send_header('Access-Control-Expose-Headers', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        return super(CORSRequestHandler, self).end_headers()
    
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, HEAD, PUT, DELETE')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.send_header('Access-Control-Allow-Credentials', 'true')
        self.send_header('Access-Control-Max-Age', '86400')
        self.send_header('Access-Control-Expose-Headers', '*')
        self.send_header('Content-Length', '0')
        self.end_headers()
    
    def do_HEAD(self):
        # Handle HEAD requests with CORS
        super().do_HEAD()
    
    def do_GET(self):
        # Handle GET requests with CORS
        super().do_GET()
    
    def do_POST(self):
        # Handle POST requests with CORS
        super().do_POST()
    
    def log_message(self, format, *args):
        # Log all requests to see what's happening
        print(f"[CORS Server] {format % args}")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""
    daemon_threads = True
    allow_reuse_address = True


if __name__ == "__main__":
    port = 8000
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    
    # Change to the directory to serve (default is current directory)
    serve_directory = '/opt/ros/humble/share/'
    if len(sys.argv) > 2:
        serve_directory = sys.argv[2]
    
    if os.path.exists(serve_directory):
        os.chdir(serve_directory)
        print(f"Starting CORS-enabled HTTP server for meshes on port {port} from {serve_directory} ...")
    else:
        print(f"Directory {serve_directory} does not exist, serving from current directory")
    
    httpd = ThreadedHTTPServer(('0.0.0.0', port), CORSRequestHandler)
    print(f"HTTP server started with CORS enabled.")
    print(f"Serving HTTP on 0.0.0.0 port {port} (http://0.0.0.0:{port}/) ...")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        httpd.shutdown() 
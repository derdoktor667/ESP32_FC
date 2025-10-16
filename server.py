import http.server
import ssl
import os

web_dir = os.path.join(os.path.dirname(__file__), 'webapp')
os.chdir(web_dir)

server_address = ('localhost', 8000)
httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile=os.path.join(os.path.dirname(__file__), 'cert.pem'), keyfile=os.path.join(os.path.dirname(__file__), 'key.pem'))
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

print("Serving HTTPS on https://localhost:8000")
httpd.serve_forever()

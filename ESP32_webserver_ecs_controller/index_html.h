const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
  <body>
    <h1>RC Controller</h1>
    <script>
      const ws = new WebSocket('ws://' + location.host + '/ws');
      ws.onopen = () => console.log('WebSocket connected');
      ws.onclose = () => console.log('WebSocket disconnected');
      ws.onmessage = (event) => console.log('Received: ' + event.data);
      const send = (param, value) => ws.send(param + '=' + value);
    </script>
  </body>
</html>
)rawliteral";

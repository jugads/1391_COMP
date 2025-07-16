const socket = new WebSocket('ws://10.13.91.21:5050'); // Use local WebSocket URL

// Wait for the WebSocket to open
socket.addEventListener('open', () => {
  console.log('WebSocket connection established!');
});

// Handle WebSocket errors
socket.addEventListener('error', (event) => {
  console.log('WebSocket error:', event);
});

// Handle WebSocket closure
socket.addEventListener('close', () => {
  console.log('WebSocket connection closed');
});

// Handle form submission
document.getElementById('dashboard-form').addEventListener('submit', function(event) {
  event.preventDefault();  // Prevent the default form submission
  
  const formData = {
    l4Scored: document.getElementById('l4-scored').value,
    l3Scored: document.getElementById('l3-scored').value,
    l2Scored: document.getElementById('l2-scored').value,
    l1Scored: document.getElementById('l1-scored').value,
    strategy: document.getElementById('strategy').value
  };

  // Check WebSocket readyState before sending data
  if (socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(formData));  // Send data only if the connection is open
    console.log('Data sent to robot:', formData);
  } else {
    console.error('WebSocket is not open. Cannot send data.');
  }
});

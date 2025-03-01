import requests

class Client:
    def __init__(self, server_url):
        """
        Initialize the client with the server URL.
        """
        self.server_url = server_url

    def send_request(self, value, question, image_path):
        """
        Sends a request to the server with a value, question, and image.

        Args:
            value (int): The value to send.
            question (str): The question to send.
            image_path (str): Path to the image file to upload.

        Returns:
            dict: The server's response.
        """
        # Create the payload for the question and value
        payload = {"value": value, "prompt": question}

        # Open the image file in binary mode
        with open(image_path, "rb") as image_file:
            files = {"image": image_file}  # Send the image file

            try:
                # Make the POST request with multipart data
                response = requests.post(self.server_url, data=payload, files=files)
                if response.status_code == 200:
                    return response.json()
                else:
                    return {"error": f"Server returned status code {response.status_code}"}
            except requests.exceptions.RequestException as e:
                return {"error": str(e)}

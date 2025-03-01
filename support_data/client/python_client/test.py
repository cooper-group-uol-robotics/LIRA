from robin_client import Client
import time
import cv2

# Define the robot movement logic
def move_robot_to_position(value):
    """
    Move the robot to a predefined position based on the value.
    Args:
        value (int): The predefined position index.
    """
    positions = {
        1: "Position A",
        2: "Position B",
        3: "Position C",
    }
    if value in positions:
        print(f"Moving robot to {positions[value]}...")
        # Simulate robot movement with a delay
        time.sleep(2)  # Replace with actual robot movement logic
        print(f"Robot has reached {positions[value]}.")
    else:
        print(f"No predefined position for value {value}. Please add it to the positions map.")

def capture_image(image_path):
    """
    Simulates capturing an image from the camera and saving it.
    Args:
        image_path (str): Path to save the captured image.
    """
    # Replace with actual image capture logic
    print("Capturing image...")
    # Simulating image capture using OpenCV
    # (Replace with your camera device and resolution)
    cap = cv2.VideoCapture(0)  # Use appropriate camera index or device path
    if not cap.isOpened():
        raise Exception("Failed to open camera.")
    ret, frame = cap.read()
    if not ret:
        raise Exception("Failed to capture image.")
    cv2.imwrite(image_path, frame)
    print(f"Image saved to {image_path}")
    cap.release()

def main():
    # Initialize the client with the server URL
    client = Client("http://172.31.1.79:65432/process")

    # Predefined list of value-question-image triplets
    queries = [
        (1, "Has the black rack been placed onto the holder properly?.", "/tmp/image1.jpg"),
        (2, "Has the white long rack been placed onto the holder properly?.", "/tmp/image2.jpg"),
    ]

    responses = []  # Store all responses for inspection

    try:
        for value, question, image_path in queries:
            # Move the robot to the predefined position
            print(f"\nStarting inspection for Value={value}, Question='{question}'")
            move_robot_to_position(value)

            # Capture an image
            capture_image(image_path)

            # Send the question and image to the server
            print(f"Sending Question: '{question}' with Image: {image_path}")
            response = client.send_request(value, question, image_path)
            print(f"Response debug: '{response}'")

            raw_response = response['response']  
            formatted_response = raw_response.split("\n")[-1].strip()
            # Store and print the response
            responses.append({"value": value, "question": question, "response": response})
            print("Response:", formatted_response)

        print("\nAll inspections have been done.")
        print("Press Enter to continue to the next step after inspection.")
        input()

        # Display all collected responses
        print("\nCollected Responses for Review:")
        for idx, entry in enumerate(responses, 1):
            raw_response = entry['response']['response'] 
            # Extract the part after the last '\n'
            formatted_response = raw_response.split("\n")[-1].strip()
            print(f"{idx}. Question: {entry['question']}, Response: {formatted_response}")

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()

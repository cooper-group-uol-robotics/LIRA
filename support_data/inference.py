import json
import difflib
from transformers import AutoProcessor, LlavaOnevisionForConditionalGeneration
import torch
from PIL import Image
import transformers

# Suppress info-level logging
transformers.logging.set_verbosity_error()

# Load the model and processor
original_model_id = "llava-hf/llava-onevision-qwen2-0.5b-ov-hf"
model_id = "./checkpoints/llava-onevision-0.5b-ov_train3"
model = LlavaOnevisionForConditionalGeneration.from_pretrained(
    model_id,
    torch_dtype=torch.float16,
    low_cpu_mem_usage=True,
).to(0)
processor = AutoProcessor.from_pretrained(original_model_id)

# Read the JSON file containing the test samples
with open('./dataset/test_data.json', 'r') as f:
    data_samples = json.load(f)

# Define a threshold for similarity; answers with similarity above this are considered correct.
SIMILARITY_THRESHOLD = 0.9

total_samples = 0
correct_samples = 0
results = []  # to store individual sample results

# Process each sample in the JSON file
for sample in data_samples:
    total_samples += 1

    conversation = []
    if "system_prompt" in sample:
        conversation.append({
            "role": "system",
            "content": [{"type": "text", "text": sample["system_prompt"]}]
        })

    # Build the conversation turns (human and assistant)
    for turn in sample.get("conversations", []):
        role = "human" if turn["from"] == "human" else "assistant"
        if turn["value"].startswith("<image>"):
            text = turn["value"][len("<image>"):].strip()
            content_list = []
            if text:
                content_list.append({"type": "text", "text": text})
            content_list.append({"type": "image"})
        else:
            content_list = [{"type": "text", "text": turn["value"].strip()}]
        conversation.append({
            "role": role,
            "content": content_list
        })

    # Create the prompt using the processor’s chat template
    prompt = processor.apply_chat_template(conversation, add_generation_prompt=True)
    
    # Load the corresponding image (assumed to be in "./dataset/resized_images/")
    image_path = f"./dataset/resized_images/{sample['image']}"
    raw_image = Image.open(image_path).convert("RGB")
    
    # Process inputs and generate model output
    inputs = processor(images=raw_image, text=prompt, return_tensors='pt').to(0, torch.float16)
    output = model.generate(**inputs, max_new_tokens=2000, do_sample=False)
    decoded_output = processor.decode(output[0][2:], skip_special_tokens=True)
    
    # Extract the assistant's answer by splitting on the token "assistant"
    parts = decoded_output.split("assistant")
    if len(parts) > 1:
        generated_answer = parts[1].strip()
    else:
        generated_answer = decoded_output.strip()
    
    # Extract ground truth answer (assuming "gpt" turns hold ground truth)
    ground_truth_list = [turn["value"].strip() for turn in sample.get("conversations", []) if turn["from"] == "gpt"]
    ground_truth = " ".join(ground_truth_list)
    
    # Compare the generated answer and ground truth using difflib
    similarity = difflib.SequenceMatcher(None, generated_answer.lower(), ground_truth.lower()).ratio()
    is_correct = similarity >= SIMILARITY_THRESHOLD
    if is_correct:
        correct_samples += 1

    # Append the result for this sample
    results.append({
        "generated": generated_answer,
        "ground_truth": ground_truth,
        "similarity": similarity,
        "correct": is_correct
    })

# Calculate overall success rate
success_rate = (correct_samples / total_samples) * 100 if total_samples > 0 else 0
final_results = {
    "overall_success_rate": success_rate,
    "correct_samples": correct_samples,
    "total_samples": total_samples,
    "samples": results
}

# Save the final results into a JSON file
with open('evaluation_results.json', 'w') as f:
    json.dump(final_results, f, indent=4)

print(f"Results saved to evaluation_results.json with overall success rate: {success_rate:.2f}%")

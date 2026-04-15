import requests
import json
import sys

import os

# Configuration
API_KEY = "e70a1849-d992-4a39-b75f-c3af033fee4b"
URL = "https://ark.cn-beijing.volces.com/api/v3/responses"
MODEL = "ep-20251208190154-944k6"
KB_DIR = r"C:\Users\yf\Desktop\人形机器人\语音交互\语音交互测试\本地知识库"

HEADERS = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json"
}

def load_knowledge_base():
    """Load all text files from the knowledge base directory."""
    kb_content = ""
    if not os.path.exists(KB_DIR):
        print(f"Warning: Knowledge base directory not found: {KB_DIR}")
        return kb_content

    print(f"Loading knowledge base from: {KB_DIR}")
    for filename in os.listdir(KB_DIR):
        if filename.endswith(".txt"):
            file_path = os.path.join(KB_DIR, filename)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    kb_content += f"\n--- Document: {filename} ---\n{content}\n"
                    print(f"Loaded: {filename}")
            except Exception as e:
                print(f"Error reading {filename}: {e}")
    return kb_content

def send_message(messages):
    payload = {
        "model": MODEL,
        "input": messages
    }
    
    # print(f"DEBUG: Sending messages: {json.dumps(messages, indent=2, ensure_ascii=False)}")
    
    try:
        response = requests.post(URL, headers=HEADERS, json=payload)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"\nError calling API: {e}")
        if hasattr(e, 'response') and e.response is not None:
            print(f"Response: {e.response.text}")
        return None

def main():
    print("--- Real-time Conversation Test (Type 'exit' or 'quit' to end) ---")
    
    # Load knowledge base
    kb_content = load_knowledge_base()
    
    # Initialize conversation history
    messages = []
    
    if kb_content:
        system_prompt = f"""你是一个智能助手。请优先基于以下本地知识库的内容回答用户的问题。
如果知识库中没有相关信息，请如实告知，并根据你的通用知识回答。

=== 本地知识库开始 ===
{kb_content}
=== 本地知识库结束 ===
"""
        # Some models prefer system prompt as the first user message if system role is not strictly enforced
        messages.append({
            "role": "user",
            "content": [{"type": "input_text", "text": system_prompt}]
        })
        messages.append({
            "role": "assistant",
            "content": [{"type": "input_text", "text": "好的，我已经阅读了本地知识库的内容。请问有什么我可以帮你的？"}]
        })
        print("Knowledge base loaded and initial context configured.")
    
    if len(sys.argv) > 1:
        initial_input = sys.argv[1]
        
        # Add to history
        messages.append({
            "role": "user",
            "content": [{"type": "input_text", "text": initial_input}]
        })
        
        print(f"You: {initial_input}")
        print("Assistant is thinking...", end="", flush=True)
        response_data = send_message(messages)
        print("\r" + " " * 30 + "\r", end="", flush=True)
        
        if response_data and 'output' in response_data:
            assistant_text = ""
            assistant_content = []
            for item in response_data['output']:
                if item.get('type') == 'message' and item.get('role') == 'assistant':
                    for content_part in item.get('content', []):
                        if content_part.get('type') == 'output_text':
                            text_chunk = content_part.get('text', '')
                            assistant_text += text_chunk
                            assistant_content.append({"type": "input_text", "text": text_chunk})
            
            if assistant_text:
                print(f"Assistant: {assistant_text}")
                messages.append({"role": "assistant", "content": assistant_content})
            else:
                print("Error: No text content found in response.")
        else:
            print("Error: Invalid response format or API failure.")

    while True:
        try:
            user_input = input("\nYou: ").strip()
        except EOFError:
            break
            
        if not user_input:
            continue
            
        if user_input.lower() in ('exit', 'quit'):
            print("Goodbye!")
            break
            
        # Construct user message
        # Using the specific format required by the endpoint based on previous successful call
        user_message = {
            "role": "user",
            "content": [
                {
                    "type": "input_text",
                    "text": user_input
                }
            ]
        }
        
        # Append to history for the API call context (if the API supports it)
        # Note: Some APIs are stateless and require full history, others might be stateful.
        # Assuming stateless (common for REST APIs), so we send full history.
        messages.append(user_message)
        
        print("Assistant is thinking...", end="", flush=True)
        response_data = send_message(messages)
        print("\r" + " " * 30 + "\r", end="", flush=True) # Clear "thinking" message
        
        if response_data and 'output' in response_data:
            # The API returns a list of outputs. Usually the last one is the response?
            # Based on previous log: "output": [{"type": "reasoning", ...}, {"type": "message", "role": "assistant", ...}]
            
            assistant_text = ""
            assistant_content = []
            
            for item in response_data['output']:
                if item.get('type') == 'message' and item.get('role') == 'assistant':
                    for content_part in item.get('content', []):
                        if content_part.get('type') == 'output_text':
                            text_chunk = content_part.get('text', '')
                            assistant_text += text_chunk
                            # Store for history
                            assistant_content.append({
                                "type": "input_text", # Note: 'output_text' from API, but when feeding back as history, usually it's 'input_text' or just 'text'? 
                                                      # Let's try to mimic the input format for history consistency.
                                "text": text_chunk
                            })
                elif item.get('type') == 'reasoning':
                    # Optional: Print reasoning if available
                    pass

            if assistant_text:
                print(f"Assistant: {assistant_text}")
                
                # Add assistant response to history
                messages.append({
                    "role": "assistant",
                    "content": assistant_content
                })
            else:
                print("Error: No text content found in response.")
                print(json.dumps(response_data, indent=2, ensure_ascii=False))
        else:
            print("Error: Invalid response format or API failure.")
            # Remove the last user message since it wasn't processed successfully
            if messages:
                messages.pop()

if __name__ == "__main__":
    main()

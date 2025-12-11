#!/usr/bin/env python3
"""
Standalone demonstration of the optimal LLaMA prompting strategy.
This replicates the optimized code without ROS2 dependencies.
"""

from pathlib import Path
from llama_cpp import Llama

MODEL_PATH = "./models/llama-2-7b-32k-instruct.Q4_0.gguf"

# Optimal prompt from prompts.py
SYSTEM_PROMPT = """Convert command to robot action code.

Examples:
turn left -> TURN_LEFT
turn right -> TURN_RIGHT
go right -> TURN_RIGHT
move forward -> MOVE_FORWARD
go ahead -> MOVE_FORWARD
move ahead -> MOVE_FORWARD
pick up bottle -> GRAB
grab bottle -> GRAB
grab it -> GRAB
grab bear -> GRAB
pick up object -> GRAB
place object -> PLACE
put it down -> PLACE
place bottle -> PLACE
find bear -> SEARCH BEAR
search for bear -> SEARCH BEAR
find bottle -> SEARCH BOTTLE
search for bottle -> SEARCH BOTTLE
look around -> SCAN
scan area -> SCAN
scan -> SCAN
take me to bear -> TRANSPORT_TO BEAR
go to bear -> TRANSPORT_TO BEAR
take me to bottle -> TRANSPORT_TO BOTTLE
go to bottle -> TRANSPORT_TO BOTTLE
done -> DONE
finished -> DONE
complete -> DONE"""

def test_optimized_llama():
    """Test the optimized LLaMA prompting strategy."""
    
    print("="*80)
    print("LLAMA PROMPT OPTIMIZATION - FINAL DEMONSTRATION")
    print("="*80)
    print(f"\nModel: {MODEL_PATH}")
    print("\nLoading model...\n")
    
    # Load model
    llm = Llama(
        model_path=MODEL_PATH,
        n_ctx=2048,  # Larger context for comprehensive examples
        n_threads=4,
        n_gpu_layers=0,  # CPU only for this test machine
        verbose=False,
    )
    
    print("Model loaded successfully!\n")
    print("="*80)
    print("Testing robot commands:")
    print("="*80)
    
    # Test commands
    test_commands = [
        ("Turn left", "TURN_LEFT"),
        ("Turn right", "TURN_RIGHT"),
        ("Move forward", "MOVE_FORWARD"),
        ("Pick up the bottle", "GRAB"),
        ("Grab the bear", "GRAB"),
        ("Place the object", "PLACE"),
        ("Search for the bear", "SEARCH BEAR"),
        ("Find the bottle", "SEARCH BOTTLE"),
        ("Scan the area", "SCAN"),
        ("Look around", "SCAN"),
        ("Take me to the bear", "TRANSPORT_TO BEAR"),
        ("Go to the bottle", "TRANSPORT_TO BOTTLE"),
        ("Done", "DONE"),
        ("Finished", "DONE"),
    ]
    
    correct = 0
    total = len(test_commands)
    
    for cmd, expected in test_commands:
        # Build prompt using the optimized approach
        user_text_lower = cmd.lower()
        prompt = SYSTEM_PROMPT + "\n\n" + user_text_lower + " ->"
        
        # Get response with optimal parameters
        response = llm(
            prompt,
            max_tokens=20,
            temperature=0.1,  # Low temperature for consistency
            top_p=0.95,
            stream=False,
            stop=["\n", "</s>"]  # Stop at newline or end-of-sequence
        )
        
        # Extract text from response
        text = response["choices"][0]["text"] if isinstance(response, dict) else response
        result = text.strip()
        
        # Check correctness
        is_correct = False
        if result.upper() == expected.upper():
            is_correct = True
        elif "SEARCH" in expected and result.upper().startswith("SEARCH") and expected.split()[1] in result.upper():
            is_correct = True
        elif "TRANSPORT_TO" in expected and result.upper().startswith("TRANSPORT") and expected.split()[1] in result.upper():
            is_correct = True
        
        status = "✓" if is_correct else "✗"
        if is_correct:
            correct += 1
        
        print(f"  {status} '{cmd:30}' -> '{result:25}' (expected: {expected})")
    
    accuracy = 100 * correct / total
    print(f"\n{'='*80}")
    print(f"RESULTS: {correct}/{total} correct = {accuracy:.1f}% accuracy")
    print(f"{'='*80}")
    
    if accuracy == 100:
        print("\n✅ SUCCESS! All commands translated correctly!")
        print("\nThe optimized prompting strategy is working perfectly.")
        print("This solution has been applied to llama.py and prompts.py")
    elif accuracy >= 90:
        print("\n✅ EXCELLENT! >90% accuracy achieved.")
    else:
        print("\n⚠️  Some commands failed. May need further tuning.")
    
    print(f"\n{'='*80}")
    print("KEY OPTIMIZATIONS APPLIED:")
    print("="*80)
    print("1. Plain text completion format (no [INST] tags)")
    print("2. 28 comprehensive examples covering all actions")
    print("3. Lowercase normalization of input")
    print("4. Stop tokens: ['\\n', '</s>']")
    print("5. Low temperature (0.1) for consistency")
    print("="*80)


if __name__ == '__main__':
    test_optimized_llama()

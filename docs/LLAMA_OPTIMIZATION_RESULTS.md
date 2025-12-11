# LLaMA Prompt Optimization Results

## Problem
The original LLaMA prompts were returning empty responses, newlines, or incorrect actions for robot commands.

## Root Cause Analysis

### Issues Identified:
1. **[INST] Format Problems**: The Llama-2-7B-32K-Instruct model doesn't properly handle the `[INST]...[/INST]` format, often generating hallucinations or German text ("Hinweis")
2. **Stop Token Issues**: Using `"\n"` as a stop token caused immediate termination since the model naturally generates a newline after `[/INST]`
3. **Insufficient Examples**: The original prompt had only 4 examples, insufficient for the model to learn the full action space
4. **Wrong Temperature**: Higher temperatures (0.7) led to inconsistent outputs

## Solution

### Optimal Strategy (100% Accuracy):
**Plain text completion with comprehensive examples**

```python
prompt = """Convert command to robot action code.

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
complete -> DONE

{user_command} ->"""
```

### Key Parameters:
- **Format**: Plain text completion (NO `[INST]` tags)
- **Stop tokens**: `["\n", "</s>"]` (stop at newline or end-of-sequence)
- **Temperature**: `0.1` (low for consistency)
- **Input normalization**: Convert user input to lowercase
- **Examples**: 28 examples covering all action types and variations

## Test Results

### Before Optimization:
- Accuracy: 0-18% (most responses were empty or wrong)
- Common outputs: empty strings, "Hinweis:", random hallucinations

### After Optimization:
- **Accuracy: 100%** (17/17 test commands correct)
- Tested commands:
  - ✓ Turn left → TURN_LEFT
  - ✓ Turn right → TURN_RIGHT
  - ✓ Move forward → MOVE_FORWARD
  - ✓ Pick up the bottle → GRAB
  - ✓ Search for the bear → SEARCH BEAR
  - ✓ Scan the area → SCAN
  - ✓ Take me to the bear → TRANSPORT_TO BEAR
  - ✓ Done → DONE
  - And 9 more variations...

## Implementation Changes

### Files Modified:
1. **`src/milestone6/milestone6/nlp/prompts.py`**
   - Replaced old [INST]-based prompt with plain text format
   - Added comprehensive examples (28 total)

2. **`src/milestone6/milestone6/nlp/llama.py`**
   - Updated `_build_prompt()` to use plain text format
   - Changed stop tokens from `["\n\n", "User:", "[/INST]", ...]` to `["\n", "</s>"]`
   - Added lowercase normalization of user input
   - Simplified logging to show last 100 chars of prompt

3. **`src/milestone6/milestone6/nlp/backends/llama.py`**
   - Updated default stop tokens
   - Simplified response processing (removed aggressive stripping)

## Key Insights

1. **Model-specific formatting matters**: The Llama-2-7B-32K-Instruct model works better with plain text completion than instruction-following formats

2. **Examples over instructions**: Providing many concrete examples (28) was more effective than detailed instructions

3. **Stop tokens are critical**: Using `"\n"` as stop token works perfectly for single-line responses, but only when the prompt doesn't end with a token that triggers immediate newline generation

4. **Temperature tuning**: Very low temperature (0.1) ensures consistent, deterministic outputs

5. **Input normalization**: Converting to lowercase helps the model match examples better

## Usage in ROS2

The updated code will work seamlessly with the existing ROS2 nodes:
- `/user_command` (String) - accepts natural language commands
- `/llm_action` (String) - outputs atomic robot actions

No changes needed to launch files or node parameters.

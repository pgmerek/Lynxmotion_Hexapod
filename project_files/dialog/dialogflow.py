import dialogflow_v2 as dialogflow
import os


def detect_intent_audio(project_id, session_id, audio_file_path,
                        language_code):
    """
    Returns response to audio file
    """
	
    session_client = dialogflow.SessionsClient()

    audio_encoding = dialogflow.enums.AudioEncoding.AUDIO_ENCODING_LINEAR_16
    sample_rate_hertz = 44100

    session = session_client.session_path(project_id, session_id)

    with open(audio_file_path, 'rb') as audio_file:
        input_audio = audio_file.read()

    audio_config = dialogflow.types.InputAudioConfig(
        audio_encoding=audio_encoding, language_code=language_code,
        sample_rate_hertz=sample_rate_hertz)
    query_input = dialogflow.types.QueryInput(audio_config=audio_config)

    response = session_client.detect_intent(
        session=session, query_input=query_input,
        input_audio=input_audio)

    return response.query_result.fulfillment_text
	
	
def detect_intent_texts(project_id, session_id, texts, language_code):
    """Returns the result of detect intent with texts as inputs.

    Using the same `session_id` between requests allows continuation
    of the conversation."""

    import dialogflow_v2 as dialogflow
    session_client = dialogflow.SessionsClient()

    session = session_client.session_path(project_id, session_id)

    for text in texts:
        text_input = dialogflow.types.TextInput(
            text=text, language_code=language_code)

        query_input = dialogflow.types.QueryInput(text=text_input)

        response = session_client.detect_intent(
            session=session, query_input=query_input)

        return response.query_result.fulfillment_text



current_directory = os.getcwd()
user_input = os.path.join(current_directory, 'weather-mono.wav')
result = detect_intent_audio("feelings-a1c4f", "1-1-1-1-1", user_input, 'en-US')
print result
result = detect_intent_texts("feelings-a1c4f", "1-1-1-1-1", ["Hey"], 'en-US')
print result

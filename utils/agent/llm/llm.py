

import os
from langchain_openai import ChatOpenAI
from langchain_ollama import ChatOllama
# from langchain_google_genai import ChatGoogleGenerativeAI

from dotenv import load_dotenv
load_dotenv()

# ros_ws = os.environ.get("ROS_WS")
# env_path = os.path.join(ros_ws, "src", "edie8_llm", ".env")
# load_dotenv(dotenv_path=env_path)

def get_llm_api(model_name="gpt-3.5-turbo",temperature=0.3):
    # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)
    
    llm = ChatOpenAI(model_name=model_name, temperature=temperature)

    return llm

def get_llm_server(model_name="qwen3:14b",temperature=0.3):
    # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

    base_url = "http://localhost:11500" 
    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    llm = ChatOllama(
        model=model_name, 
        temperature=temperature,
        base_url = base_url,
        )
    return llm

def get_llm_local(model_name="qwen2.5:0.5b", num_thread=4, temperature=0.3):
    # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    llm = ChatOllama(
        model=model_name, 
        temperature=temperature,
        num_gpu=-1,
        num_thread=num_thread
        )
    return llm


def get_llm(local=False, model_name="qwen2.5:0.5b", num_thread=4, temperature=0.3):
    # llm = ChatOllama(model="qwen3:1.7b", temperature=temperature)

    # model_name = "qwen3:14b" # "qwen3:8b" # "qwen3:4b" "qwen3:1.7b"
    
    if local:
        llm = ChatOllama(
            model=model_name, 
            temperature=temperature,
            num_gpu=-1,
            num_thread=num_thread
            )
    else:
        llm = ChatOpenAI(model_name=model_name, temperature=temperature)
        # llm = ChatGoogleGenerativeAI(model="gemini-2.0-flash", temperature=temperature)
        # gemini-2.5-pro-preview-03-25, gemini-2.5-flash-preview-04-17
    
    return llm

def get_env_variable(var_name: str) -> str:
    value = os.getenv(var_name)
    if value is None:
        raise ValueError(f"Environment variable {var_name} is not set.")
    return value
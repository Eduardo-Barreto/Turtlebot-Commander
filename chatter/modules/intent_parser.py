import json
from typing_extensions import TypedDict
from unicodedata import normalize

import google.generativeai as genai


def normalize_text(text: str) -> str:
    return normalize("NFKD", text.lower()).encode("ASCII", "ignore").decode("ASCII")


class Response(TypedDict):
    destination: str
    response: str


class GoalExtractor:
    def __init__(self, api_key: str, locations_file: str):
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel("gemini-1.5-flash")

        with open(locations_file) as f:
            self.locations = json.load(f)

    def extract_goal(self, command: str) -> Response:
        result = self.model.generate_content(
            "Você é um assistente virtual que responde em português e auxilia na interação com um robô. Preciso que você entenda onde o usuário deseja ir. Mensagem do usuário: "
            + command
            + "Localizações disponíveis: "
            + ", ".join(self.locations)
            + ". Responda amigavelmente e com uma pitadinha de sarcasmo (de vez em quando) ao usuário. ",
            generation_config=genai.GenerationConfig(
                response_mime_type="application/json", response_schema=Response
            ),
        ).text

        response = json.loads(result)
        return response

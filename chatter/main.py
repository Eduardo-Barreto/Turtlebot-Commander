from google.generativeai.types.generation_types import json
from modules.goal_mapper import GoalMapper
from modules.ros_client import ROSClient
from modules.intent_parser import GoalExtractor
from yaspin import yaspin
from rich.console import Console
from rich.prompt import Prompt
import rclpy
import os
from dotenv import load_dotenv

load_dotenv()

console = Console()
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")


def main():
    rclpy.init()

    console.print("[bold]Destinos disponíveis:[/bold]")

    with open("config/locations.json") as f:
        locations = json.load(f)
        for location in locations:
            console.print(f"- {location}")
    console.print("[bold magenta]> E aí, meu querido, o que manda?[/bold magenta]")
    goal_extractor = GoalExtractor(GEMINI_API_KEY, "config/locations.json")
    mapper = GoalMapper("config/locations.json")
    ros_client = ROSClient()

    try:
        while True:
            command = Prompt.ask("[bold blue]Você[/bold blue]")
            response = goal_extractor.extract_goal(command)
            destination = response["destination"]
            response_text = response["response"]

            coords = mapper.get_coordinates(destination)
            if coords is None:
                console.print(f"[bold red]{response_text}[/bold red]")
                continue

            console.print(f"[bold green]{response_text}[/bold green]")
            x, y, yaw = coords["x"], coords["y"], coords["rotation"]
            with yaspin(text=f"Indo para {destination}...", color="cyan") as spinner:
                success = ros_client.send_goal(x, y, yaw)
                if success:
                    spinner.text = f"Chegamos ao destino: {destination}!"
                    spinner.ok("✅")
                else:
                    spinner.text = f"Erro ao tentar ir para {destination}."
                    spinner.fail("❌")
                    console.print(
                        f"[bold red]Falha ao atingir o destino {destination}.[/bold red]"
                    )
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Encerrando CLI...[/bold yellow]")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

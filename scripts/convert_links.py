import os
import re

vault_path = r"C:\Users\inigo\Universidad Politécnica de Madrid\INGENIA - Robots y AGVs (2024-25) - General\Ingenia Alumnos\04 - Arquitectura\0402 - Integración\Documentacion"

# Recorrer los archivos .md
for root, dirs, files in os.walk(vault_path):
    for filename in files:
        if filename.endswith(".md"):
            filepath = os.path.join(root, filename)
            with open(filepath, "r", encoding="utf-8") as f:
                content = f.read()

            # Reemplazar [[Nombre]] o [[Nombre|Alias]]
            content_new = re.sub(
                r"\[\[([^\]\|]+)\|?([^\]]+)?\]\]",
                lambda m: f"[{m.group(2) or m.group(1)}]({m.group(1)}.md)",
                content,
            )

            if content != content_new:
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(content_new)
                print(f"Links actualizados en {filepath}")


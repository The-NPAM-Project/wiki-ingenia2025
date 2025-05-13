#!/usr/bin/env python3
# La línea de arriba es IMPRESCINDIBLE para que funcionen C++ y Python juntos

# Archivo: interface.py
# Versión: V1.0
# Autores: Miguel Ángel García de Vicente


import threading
import rclpy
from rclpy.node import Node
from mastermind.interface_node import InterfaceNode
import tkinter as tk
from mastermind.interface_window import InterfaceWindow


def main(args=None):
    rclpy.init(args=args)

    # Crea el nodo y lo pone en un thread a parte para que pueda funcionar
    # en paralelo con la ventana de Tkinter
    interfaceNode = InterfaceNode()
    thread_spin = threading.Thread(target=rclpy.spin, args=(interfaceNode, ))
    thread_spin.start()

    #
    root = tk.Tk()
    app = InterfaceWindow(root)
    root.mainloop()

    interfaceNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
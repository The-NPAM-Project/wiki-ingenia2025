# Archivo: interface_window.py
# Versión: V1.0
# Autores: Miguel Ángel García de Vicente


import tkinter as tk
import customtkinter as ct
from tkinter import filedialog, Label
import webbrowser
from PIL import Image, ImageTk
from ament_index_python.packages import get_package_share_directory
import os

# La siguiente librería solo funciona en windows
try:
    from ctype import windll, byref, sizeof, c_int
except:
    pass

class InterfaceWindow:
    def __init__(self, root):
        self.root = root
        self.rutaArchivo = ""

        # Directorio del archivo actual (será el compilado, no este)
        self.package_path = get_package_share_directory('mastermind')
        self.package_path = self.package_path[:-36] + "/src/mastermind"

        # Colores
        self.gris_claro = "#D6CCCC"
        self.azul_saturado = "#009FE3"
        self.gris_oscuro = "#878786"
        self.gris_oscuro2 = "#696968"
        self.color_fondo = "#FFFFFF"

        # Shortcuts
        root.bind("<Control-o>", self.keyCtrlO)
        root.bind("<Control-w>", self.keyCtrlW)

        self.setupWindow()
        self.setupWidgets()
        self.setupColocacion()

    def setupWindow(self):
        self.root.geometry(str(int(self.root.winfo_screenwidth() / 2)) + "x" + str(int(self.root.winfo_screenheight() / 2)))
        self.root.state("normal")
        self.root.minsize(640, 360)
        self.root.configure(background=self.color_fondo)
        tk.Wm.wm_title(self.root, "Mastermind SpinPrinter")

        try:
            self.root.iconphoto(True, tk.PhotoImage(file=self.package_path + "/images/minilogo.png"))
        except:
            pass

        try:
            self.root.update()
            HWND = windll.user32.GetParent(self.root.winfo_id())
            windll.dwmapi.DwmSetWindowAttribute(HWND, 35, byref(c_int(0x00878786)), sizeof(c_int))
            windll.dwmapi.DwmSetWindowAttribute(HWND, 36, byref(c_int(0x00EEEEEE)), sizeof(c_int))
        except:
            pass

        try:
            windll.dwmapi.DwmSetWindowAttribute(HWND, 20, byref(c_int(1)), sizeof(c_int))
        except:
            pass

    def setupWidgets(self):
        self.barraH = tk.Menu(self.root, background=self.gris_claro, fg="#000000")
        self.submenu1 = tk.Menu(self.barraH, tearoff=0, bg=self.gris_claro, fg="#000000")
        self.submenu2 = tk.Menu(self.barraH, tearoff=0, bg=self.gris_claro, fg="#000000")
        self.submenu3 = tk.Menu(self.barraH, tearoff=0, bg=self.gris_claro, fg="#000000")
        self.barraH.add_cascade(menu=self.submenu1, label="Archivo")
        self.barraH.add_cascade(menu=self.submenu2, label="Opciones")
        self.barraH.add_cascade(menu=self.submenu3, label="Ayuda")

        self.submenu1.add_command(label="Abrir",
                                  command=self.cmdArchivoAbrir,
                                  underline=0,
                                  compound=tk.LEFT,
                                  accelerator="Ctrl+O")
        self.submenu1.add_command(label="Cerrar",
                                  command=self.cmdArchivoCerrar,
                                  underline=0, compound=tk.LEFT,
                                  accelerator="Ctrl+W")
        self.submenu1.add_separator()
        self.submenu1.add_command(label="Salir",
                                  command=self.root.quit,
                                  underline=0,
                                  compound=tk.LEFT)

        self.submenu3.add_command(label="Repositorio",
                                  command=self.cmdAyudaRepositorio,
                                  underline=0,
                                  compound=tk.LEFT)
        self.submenu3.add_command(label="Descanso...",
                                  command=self.cmdAyudaDescanso,
                                  underline=0,
                                  compound=tk.LEFT)

        self.frameInicio = tk.Frame(self.root,
                                    bg=self.color_fondo,
                                    width=300,
                                    height=200)
        self.frameMenu = tk.Frame(self.root,
                                  bg=self.color_fondo)

        logo = Image.open(self.package_path + "/images/logotipo.png")
        logo = logo.resize((564, 610))
        self.logo = ImageTk.PhotoImage(logo)

        self.btnInicio = tk.Button(self.frameInicio,
                                   bg=self.gris_claro,
                                   image=self.logo,
                                activebackground=self.gris_claro,
                                activeforeground=self.gris_claro,
                                command=self.cmdInicio,
                                relief="flat",
                                borderwidth=0)

        self.frameMenuOpciones = tk.Frame(self.frameMenu,
                                          bg=self.gris_claro,
                                          width=300)
        self.frameMenuControl = tk.Frame(self.frameMenu,
                                         bg=self.gris_claro,
                                         width=300,
                                         height=70)
        self.frameInfo = tk.Frame(self.frameMenu,
                                  bg=self.gris_claro)

        self.frameBtnImprimir = tk.Frame(self.frameMenuControl,
                                         width=((self.frameMenuControl.winfo_reqwidth() - 30) / 2),
                                         height=50)
        self.btnImprimir = tk.Button(self.frameBtnImprimir,
                                     text="Imprimir",
                                     font=("Aptos", 12),
                                     bg=self.gris_oscuro,
                                     fg="#EEEEEE",
                                     activebackground=self.gris_oscuro2,
                                     activeforeground="#EAEAEA",
                                     command=self.cmdImprimir,
                                     width=250,
                                     height=50,
                                     relief="flat",
                                     borderwidth=0)

        self.frameBtnParada = tk.Frame(self.frameMenuControl, width=((self.frameMenuControl.winfo_reqwidth() - 30) / 2), height=50)
        self.btnParada = tk.Button(self.frameBtnParada, text="Parada",
                                   font=("Aptos", 12),
                                   bg=self.gris_oscuro,
                                   fg="#EEEEEE",
                                   activebackground=self.gris_oscuro2,
                                   activeforeground="#EAEAEA",
                                   command=self.cmdParada,
                                   width=250,
                                   height=50,
                                   relief="flat",
                                   borderwidth=0)
    
    def setupColocacion(self):
        self.root.config(menu=self.barraH)

        self.frameInicio.grid_propagate(False)
        self.frameInicio.pack(fill="both", expand=True)
        self.btnInicio.pack(fill="both", expand=True)

        self.frameMenu.columnconfigure(1, weight=1)
        self.frameMenu.rowconfigure(0, weight=1)

        self.frameMenuOpciones.grid_propagate(False)
        self.frameMenuOpciones.grid(row=0, column=0, padx=10, pady=(10, 0), sticky=tk.S + tk.N + tk.W)

        self.frameMenuControl.grid_propagate(False)
        self.frameMenuControl.grid(row=1, column=0, padx=10, pady=10, sticky=tk.S + tk.W)
        self.frameMenuControl.columnconfigure((0, 1), weight=1)
        self.frameMenuControl.rowconfigure(0, weight=1)

        self.frameBtnImprimir.pack_propagate(False)
        self.frameBtnImprimir.grid(row=0, column=0, padx=(10, 0), pady=10)
        self.btnImprimir.pack(expand=True)

        self.frameBtnParada.pack_propagate(False)
        self.frameBtnParada.grid(row=0, column=1, padx=(10), pady=10)
        self.btnParada.pack(expand=True)

        self.frameInfo.grid_propagate(False)
        self.frameInfo.grid(row=0, column=1, padx=(0, 10), pady=10, sticky=tk.N + tk.S + tk.W + tk.E, rowspan=2)

    def cmdArchivoAbrir(self):
        rutaArchivoTemp = str(filedialog.askopenfilename(title="Abrir archivo",
                                                         filetypes=(("Archivos de texto", "*.txt"), ("Todos los archivos", "*.*"))))
        if rutaArchivoTemp == "":
            return

        self.rutaArchivo = rutaArchivoTemp
        archivo = open(self.rutaArchivo, 'r')
        # -------------------
        archivo.close()

    def cmdArchivoCerrar(self):
        # -------------------
        self.rutaArchivo = ""

    def cmdAyudaRepositorio(self):
        webbrowser.open("https://github.com/The-NPAM-Project")

    def cmdAyudaDescanso(self):
        webbrowser.open("https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstley")

    def keyCtrlO(self, e):
        self.cmdArchivoAbrir()

    def keyCtrlW(self, e):
        self.cmdArchivoCerrar()

    def cmdInicio(self):
        self.frameInicio.pack_forget()
        self.frameMenu.pack(fill="both", expand=True)

    def cmdImprimir(self):
        return

    def cmdParada(self):
        os.system("ros2 lifecycle set super_logger shutdown")
        os.system("ros2 lifecycle set super_logger_launcher shutdown")
        os.system("ros2 lifecycle set cartesian_path_node shutdown")
        os.system("ros2 lifecycle set action_client_node shutdown")
        os.system("ros2 lifecycle set gcode_parser_node shutdown")
        os.system("ros2 lifecycle set bed_controller_node shutdown")
        os.system("ros2 lifecycle set extruder_controller_node shutdown")
        os.system("ros2 lifecycle set Master_node shutdown")

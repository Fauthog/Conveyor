import tkinter
import tkinter.messagebox
import customtkinter
import os
import time
from datetime import datetime
import threading
from PIL import Image
import cv2
# import A1_driver
# from picamera2 import Picamera2



customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
       
        
        # configure window
        self.title("HSF Lab Multi Tool.py")
        self.geometry(f"{1920}x{900}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure((0, 1), weight=1)

        # create tabview
        self.tabview = customtkinter.CTkTabview(self, width=1920, height=900)
        self.tabview.grid(row=0, column=0)   
        self.tabview.add("GUI")   
        self.tabview.add("Options")  


        self.tabview.tab("GUI").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs

        self.tabview.tab("Options").grid_columnconfigure(0, weight=1)

        image_path = os.path.dirname(os.path.realpath(__file__))
        logo = customtkinter.CTkImage(Image.open(os.path.join(image_path, "Logo.png")), size=(138, 63))

        self.GUI_Tab = GUI_TabFrame(self.tabview.tab("GUI"), logo)
        self.GUI_Tab.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="sw")

        self.Option_Tab = Options_TabFrame(self.tabview.tab("Options"), logo)
        self.Option_Tab.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="sw")

class GUI_TabFrame(customtkinter.CTkFrame):
    def __init__(self, master, logo):
        super().__init__(master)

        self.GUI_frame = customtkinter.CTkFrame(self)
        self.GUI_frame.grid(row=0, column=3, padx=10, pady=(10, 0), sticky="sw")

        
        self.logo_image = logo
        self.navigation_frame_label = customtkinter.CTkLabel(self.GUI_frame, image=self.logo_image, text="")
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20)

        self.start_label = customtkinter.CTkButton(self.GUI_frame, text="Start", anchor="s")
        self.start_label.grid(row=3, column=0, padx=20, pady=(10, 0))

        self.stop_label = customtkinter.CTkButton(self.GUI_frame, text="Stop", anchor="s")
        self.stop_label.grid(row=3, column=1, padx=20, pady=(10, 0))

        self.reset_label = customtkinter.CTkButton(self.GUI_frame, text="Reset", anchor="s")
        self.reset_label.grid(row=3, column=2, padx=20, pady=(10, 0))

class Options_TabFrame(customtkinter.CTkFrame):
    def __init__(self, master, logo):
        super().__init__(master)

        self.Option_frame = customtkinter.CTkFrame(self)
        self.Option_frame.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="sw")

        
        self.logo_image = logo
        self.navigation_frame_label = customtkinter.CTkLabel(self.Option_frame, image=self.logo_image, text="")
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20)

        self.appearance_mode_label = customtkinter.CTkLabel(self.Option_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.Option_frame, values=["Light", "Dark", "System"],
                                                                       command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=2, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.Option_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.Option_frame, values=["80%", "90%", "100%", "110%", "120%"],
                                                               command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=4, column=0, padx=20, pady=(10, 20))
        self.scaling_optionemenu.set("100%")

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)
       


if __name__ == "__main__":
    app = App()
    app.mainloop()
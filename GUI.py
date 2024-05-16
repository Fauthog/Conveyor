import tkinter
import tkinter.messagebox
import customtkinter
import os
import time
from PIL import Image
from threading import Thread
import conveyor_controller



customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
       
        
        # configure window
        self.title("GUI")
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

        self.StopThread:bool = False
        self.currentFrame_thread = None
        self.StopState:bool = False
        self.On:bool = False
       
        self.logo_frame = customtkinter.CTkFrame(self)
        self.logo_frame.grid(row=0, column=0, padx=10, pady=(10, 0), sticky="sw")

        
        self.logo_image = logo
        self.navigation_frame_label = customtkinter.CTkLabel(self.logo_frame, image=self.logo_image, text="Logo")
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.Preview_frame = customtkinter.CTkFrame(self, height = 500)
        self.Preview_frame.grid(row=1, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.preview_frame_label = customtkinter.CTkLabel(self.Preview_frame,  text="Preview")
        self.preview_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")
        self.preview_frame_image = customtkinter.CTkLabel(self.Preview_frame, width=240, text="")
        self.preview_frame_image.grid(row=1, column=0, padx=(20,20), pady=(20,20))

        self.control_frame = customtkinter.CTkFrame(self)
        self.control_frame.grid(row=2, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.control_frame_label = customtkinter.CTkLabel(self.control_frame,  text="control")
        self.control_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")
        
        self.start_label = customtkinter.CTkButton(self.control_frame, text="Start", anchor="s", command=self.start_callback)
        self.start_label.grid(row=3, column=0, padx=20, pady=(10, 0))

        self.stop_label = customtkinter.CTkButton(self.control_frame, text="Stop", anchor="s", command=self.stop_callback)
        self.stop_label.grid(row=3, column=1, padx=20, pady=(10, 0))

        self.reset_label = customtkinter.CTkButton(self.control_frame, text="Reset", anchor="s", command=self.reset_callback)
        self.reset_label.grid(row=3, column=2, padx=20, pady=(10, 0))

        self.OnOff_label = customtkinter.CTkButton(self.control_frame, text="On", anchor="s", command=self.OnOff_callback)
        self.OnOff_label.grid(row=3, column=3, padx=20, pady=(10, 0))

        self.scaling_label = customtkinter.CTkLabel(self.control_frame, text="Speed", anchor="w")
        self.scaling_label.grid(row=4, column=0, padx=20, pady=(10, 0))

        self.speed = customtkinter.CTkOptionMenu(self.control_frame, values=["10%","50%","100%"],
                                                               command=self.speed_event)
        self.speed.grid(row=5, column=0, padx=20, pady=(10, 20))
        self.speed.set("100%")

        self.mode_label = customtkinter.CTkLabel(self.control_frame, text="Mode", anchor="w")
        self.mode_label.grid(row=4, column=1, padx=20, pady=(10, 0))

        self.continues = customtkinter.CTkCheckBox(self.control_frame, text="continues", command=self.mode_event)
        self.continues.grid(row=5, column=1, padx=20, pady=(10, 0))

        self.state_frame = customtkinter.CTkFrame(self, width=200)
        self.state_frame.grid(row=1, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.state_frame_label = customtkinter.CTkLabel(self.state_frame,  text="state", width=500)
        self.state_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.current_state_label = customtkinter.CTkLabel(self.state_frame,  text="current state")
        self.current_state_label.grid(row=1, column=0, padx=20, pady=20, sticky="nsew")
        

        self.governor = conveyor_controller.governor()
        #self.governor = None
        self.start_label.configure(state="disabled")
        self.stop_label.configure(state="disabled")
        self.reset_label.configure(state="disabled")

    def getCurrentFrame(self):
        image = None
        while True:
                if self.StopThread:
                        return
                time.sleep(0.1)
                #print("state", self.governor.state)
                _i = self.governor.convertedImage
                if _i != image:
                    image=_i
                    if image is not None:                
                            _image = customtkinter.CTkImage(light_image=image,
                                        size=(320, 240))
                            self.preview_frame_image.configure(image = _image)
                state = self.governor.state
                if state == "No Shrimp":
                    self.current_state_label.configure(fg_color="red")
                else:
                    self.current_state_label.configure(fg_color="transparent")
                    
                self.current_state_label.configure(text = state)

    def update(self):
        self.StopThread = False
        self.currentFrame_thread = Thread(target = self.getCurrentFrame)
        self.currentFrame_thread.start()

    def speed_event(self, speed_value:str):
        speed = float(speed_value.replace("%", "")) / 100
        self.governor.setSpeed(speed)

    def mode_event(self):
        if self.continues.get==1:
            self.governor.setMode("continues")
        else:
            self.governor.setMode("single")

    def start_callback(self):
        self.governor.start_controller()
        

    def stop_callback(self):
        if not self.StopState:
            self.governor.stop_controller()
            self.stop_label.configure(text = "resume")
            self.StopState=True
        else:
            self.governor.resume_controller()
            self.stop_label.configure(text = "stop")
            self.StopState=False
        
    
    def reset_callback(self):
        self.governor.reset()

    def OnOff_callback(self):
        if not self.On:
                self.On=True
                self.OnOff_label.configure(text = "Off")
                self.start_label.configure(state="enabled")
                self.stop_label.configure(state="enabled")
                self.reset_label.configure(state="enabled")
                self.governor.OnOff("On")
                self.update()          
                
        else:   
                self.On = False
                self.OnOff_label.configure(text = "On")
                self.start_label.configure(state="disabled")
                self.stop_label.configure(state="disabled")
                self.reset_label.configure(state="disabled")
                time.sleep(0.1)
                self.governor.OnOff("Off")
                self.StopThread = True
                #if self.currentFrame_thread is not None:
                #       self.currentFrame_thread.join()
                        

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

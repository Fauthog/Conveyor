import tkinter
import tkinter.messagebox
import customtkinter
import os
import time
from PIL import Image
from threading import Thread
import conveyor_controller2



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
        self.Pause:bool = False
        self.On:bool = False
        self.Mode:str = "single"
       
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

        self.pauseResume_label = customtkinter.CTkButton(self.control_frame, text="Pause", anchor="s", command=self.pauseResume_callback)
        self.pauseResume_label.grid(row=3, column=1, padx=20, pady=(10, 0))

        self.reset_label = customtkinter.CTkButton(self.control_frame, text="Reset", anchor="s", command=self.reset_callback)
        self.reset_label.grid(row=3, column=2, padx=20, pady=(10, 0))
        
        self.mode_btn = customtkinter.CTkButton(self.control_frame, text="Mode", anchor="w", command=self.mode_callback)
        self.mode_btn.grid(row=3, column=3, padx=20, pady=(10, 0))

        self.mode_label = customtkinter.CTkLabel(self.control_frame, text=self.Mode)
        self.mode_label.grid(row=3, column=4, padx=20, pady=(10, 0))

        
        self.state_frame = customtkinter.CTkFrame(self, width=200)
        self.state_frame.grid(row=1, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.state_frame_label = customtkinter.CTkLabel(self.state_frame,  text="state", width=500)
        self.state_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.states=["init", "limit", "start", "synchronize", "pickup","detection", "analyze", "cut", "drop off", "return"]
        self.labels_0=[]
        self.labels_1=[]
    
        self.label_0 = customtkinter.CTkLabel(self.state_frame,  text="state 0")
        self.label_0.grid(row=1, column=0, padx=20, pady=20, sticky="nsew")
        for i in range(len(self.states)):
            label = customtkinter.CTkLabel(self.state_frame,  text=self.states[i])
            label.grid(row=i+2, column=0, padx=20, pady=20, sticky="nsew")
            self.labels_0.append(label)

        self.label_1 = customtkinter.CTkLabel(self.state_frame,  text="state 1")
        self.label_1.grid(row=1, column=1, padx=20, pady=20, sticky="nsew")
        for j in range(len(self.states)):
            label = customtkinter.CTkLabel(self.state_frame,  text=self.states[i])
            label.grid(row=i+2, column=1, padx=20, pady=20, sticky="nsew")
            self.labels_1.append(label)

        self.error_label = customtkinter.CTkLabel(self.state_frame)
        self.label_1.grid(row=12, column=0, padx=20, pady=20, sticky="nsew")
        

        self.governor = conveyor_controller2.governor()
        self.update()

 

    def getCurrentFrame(self):
        image = None
        while True:
                if self.StopThread:
                        return
                time.sleep(0.1)
               
                _i = self.governor.convertedImage
                if _i != image:
                    image=_i
                    if image is not None:                
                            _image = customtkinter.CTkImage(light_image=image,
                                        size=(320, 240))
                            self.preview_frame_image.configure(image = _image)
             
    

       
    def highlightlabel(self, labels, state:str)->None:
        for label in labels:
            label.configure(fg_color="transparent")
        try:
            index = labels.index(state)
            labels[index].configure(fg_color="green")
        except:
            print("no index corresponding to state found")

    def showError(self, error:str):
        if error != "":
            self.error_label.configure(fg_color="transparent", text=error)
        else:
            self.error_label.configure(fg_color="red", text=error)

    def getCurrentState(self):
        state0:str = ""
        _state0:str = ""
        state1:str = ""
        _state1:str = ""
        error:str = ""
        _error:str= ""
        while True:
            state0, state1, error = self.governor.getCurrentState()
            if state0 != _state0:
                self.highlightlabel(self.labels_0, state0)
                _state0 = state0[:]
            if state1 != _state1:
                self.highlightlabel(self.labels_1, state1)
                _state1 = state1[:]
            if error != _error:
                self.showError(error)
                _error = error[:]
            time.sleep(0.05)

    def update(self):
        self.StopThread = False
        self.currentFrame_thread = Thread(target = self.getCurrentFrame)
        self.currentFrame_thread.start()
        self.currentState_thread = Thread(target = self.getCurrentState)
        self.currentState_thread.start()
    
    def start_callback(self):
        self.governor.userInput("start")

    def pauseResume_callback(self):
        if self.Pause:
            self.governor.userInput("hold")
            self.start_label.configure(text="resume")
            self.StartState = False
            
        else:
            self.governor.userInput("resume")
            self.start_label.configure(text="hold")
            self.StartState = True

    def mode_callback(self):
        if self.Mode == "single":
            self.Mode = "cycle"
            self.mode_label.configure(text=self.Mode)
            self.governor.userInput("cycle")
        else:
            self.Mode = "single"
            self.mode_label.configure(text=self.Mode)
            self.governor.userInput("single")
              

    def reset_callback(self):
        self.governor.userInput("reset")

    
                        

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

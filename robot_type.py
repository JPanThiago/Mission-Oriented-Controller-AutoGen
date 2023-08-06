"""
Choose the robot type for optimization

"""

import tkinter as tk

def show_custom_dialog():
    root = tk.Tk()
    root.withdraw()
    dialog = tk.Toplevel()
    dialog.title('Choose robot type')

    # Set the dialog window size
    dialog.geometry('380x320')

    # Get the screen size
    screen_width = dialog.winfo_screenwidth()
    screen_height = dialog.winfo_screenheight()

    # Calculate the center position of the dialog window
    x = int((screen_width - dialog.winfo_reqwidth()) / 2)
    y = int((screen_height - dialog.winfo_reqheight()) / 2)

    # Set the center position of the dialog window
    dialog.geometry(f"+{x}+{y}")

    # Option buttons
    def handle_option(option):
        selected_option.set(option)  # Assign value to the parameter
        dialog.destroy()  # Close the window

    option_frame = tk.Frame(dialog)
    option_frame.pack(pady=20)  # Add vertical spacing

    options = ['Damping Pendulum', 'Tendon-Driven Visual Pan-Tilt', 'Soft Robot', 'Autonomous Underwater Vehicles', 'Autonomous Underwater Vehicles for the 3D Task', 'Robotic Penguin', 'Robotic Penguin with the Unidentified Six-State Model']
    selected_option = tk.StringVar()

    for option in options:
        button = tk.Button(option_frame, text=option, command=lambda opt=option: handle_option(opt))
        button.pack(pady=5)  # Add spacing between buttons

    dialog.wait_window(dialog)  # Block the code until the dialog is closed

    if selected_option.get() == 'Damping Pendulum':
        robot_type = 'DP'
    elif selected_option.get() == 'Tendon-Driven Visual Pan-Tilt':
        robot_type = 'TDVPT'
    elif selected_option.get() == 'Soft Robot':
        robot_type = 'SoftRobot'
    elif selected_option.get() == 'Autonomous Underwater Vehicles':
        robot_type = 'AUV'
    elif selected_option.get() == 'Autonomous Underwater Vehicles for the 3D Task':
        robot_type = 'AUV_3D'
    elif selected_option.get() == 'Robotic Penguin':
        robot_type = 'RoboticPenguin'
    else:
        robot_type = 'RoboticPenguin_full'
    return robot_type

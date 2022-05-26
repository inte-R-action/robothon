#!/usr/bin/env python3.7

import tkinter as Tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import numpy as np
import time
import datetime
from PIL import Image, ImageTk
from sympy import E, EX
import os
import pandas as pd
from tkinter import ttk
import argparse
import threading
# from database_funcs import database

os.chdir(os.path.dirname(os.path.realpath(__file__)))

plt.ion()

QUIT = False
possible_actions = ["Look for objects", "Red button", "Blue button", "key", "AA batteries", "Ethernet", "Cell battery", "None"]


class new_user_dialogue(object):
    def __init__(self, parent):
        self.fcancel = False
        self.toplevel = Tk.Toplevel(parent)
        self.toplevel.geometry('350x150')
        self.toplevel.resizable(False, False)
        self.var = Tk.StringVar()

        db = database()
        col_names, users = db.query_table('users', 'all')
        users = pd.DataFrame(users, columns=col_names)
        self.default = "Select User"
        options = [self.default]
        for _, row in users.iterrows():
            options.append((row['user_id'], row['user_name']))

        options = tuple(options)
        label = Tk.Label(self.toplevel, text="Choose New User:", height=1, padx=50, pady=2, anchor='center')
        om = ttk.OptionMenu(self.toplevel, self.var, options[0], *options)
        ok_button = Tk.Button(self.toplevel, text="OK", command=self.toplevel.destroy, width=10, height=1, padx=30, pady=10, anchor='center')
        cancel_button = Tk.Button(self.toplevel, text="Cancel", command=self.cancel, width=10, height=1, padx=30, pady=10, anchor='center')

        label.grid(row=0, column=0, columnspan=2)#, sticky="nsew")
        om.grid(row=1, column=0, columnspan=2)#, sticky="nsew")
        ok_button.grid(row=2, column=0)#, sticky="nsew")
        cancel_button.grid(row=2, column=1)#, sticky="nsew")

        # Adjust spacing of objects
        self.toplevel.grid_columnconfigure(0, weight=1)
        self.toplevel.grid_columnconfigure(1, weight=1)
        self.toplevel.grid_rowconfigure(0, weight=1)
        self.toplevel.grid_rowconfigure(1, weight=1)
        self.toplevel.grid_rowconfigure(2, weight=1)

    def cancel(self):
        self.fcancel = True
        self.toplevel.destroy()

    def show(self):
        self.toplevel.deiconify()
        self.toplevel.wait_window()
        value = self.var.get()
        if self.fcancel or (value == self.default):
            return None
        else:
            return value


class shutting_down_window():
    def __init__(self):
        self.Window_Width=800
        self.Window_Height=600
        self.im_size = 100
        self.xinc = self.yinc = 4

        # Create GUI
        self.root = Tk.Tk()
        self.root.wm_title("Shutting Down...")
        self.root.geometry(f'{self.Window_Width}x{self.Window_Height}')

        l = Tk.Label(self.root, text = "Dreaming State")
        l.config(font =("Courier", 20))
        l.pack(fill=Tk.BOTH)

        self.canvas = Tk.Canvas(self.root)
        self.canvas.configure(bg="Blue")
        self.canvas.pack(fill="both", expand=True)
        self.animate_ball()

    def shutdown(self):
        self.root.quit()
        self.root.destroy()

    def animate_ball(self):
        image = Image.open("dreaming.png")
        image = image.resize((self.im_size, self.im_size), Image.ANTIALIAS)
        self.the_image = ImageTk.PhotoImage(image)
        self.image = self.canvas.create_image(400, 300, anchor=Tk.NW, image=self.the_image)
        self.animate_window()

    def animate_window(self):
        self.canvas.move(self.image, self.xinc, self.yinc)
        self.root.update()
        ball_pos = self.canvas.coords(self.image)
        # unpack array to variables
        x, y = ball_pos
        if x < abs(self.xinc) or (x+self.im_size) > self.Window_Width-abs(self.xinc):
            self.xinc = -self.xinc
        if y < abs(self.yinc) or (y+self.im_size) > self.Window_Height-abs(self.yinc):
            self.yinc = -self.yinc


class GUI:
    def __init__(self, cmd_publisher):
        # self.db = database()
        # Create GUI
        self.root = Tk.Tk()
        # self.root = Toplevel
        self.root.wm_title("Robothon interaction screen")
        self.root.resizable(True, True)
        self.cmd_publisher = cmd_publisher

        self.create_header_frame()
        self.create_control_frame()
        self.create_localisation_frame()

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=1)

    def create_header_frame(self):
        self.header_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.header_frame.grid(row=0, column=0, columnspan=2, sticky="nsew")

        # Uni logo
        load = Image.open("logo.jpg")
        imsize = 100
        resized = load.resize((imsize, imsize), Image.ANTIALIAS)
        render = ImageTk.PhotoImage(image=resized)
        self.img = Tk.Label(master=self.header_frame, image=render)
        self.img.image = render
        self.img.grid(row=0, column=1)

        # Team text
        self.team_text = Tk.Text(master=self.header_frame, height=1)
        self.team_text.grid(row=0, column=0, sticky="nsew")
        self.team_text.insert(Tk.INSERT, "Team inte-R-action")
        self.team_text.config(state='disabled')

        # Competition text
        self.comp_text = Tk.Text(master=self.header_frame, height=1)
        self.comp_text.grid(row=0, column=2, sticky="nsew")
        self.comp_text.insert(Tk.INSERT, "Robothon 2022")
        self.comp_text.config(state='disabled')

        # Adjust spacing of objects
        self.header_frame.grid_columnconfigure(0, weight=1)
        self.header_frame.grid_columnconfigure(1, weight=1)
        self.header_frame.grid_columnconfigure(2, weight=1)

        self.header_frame.grid_rowconfigure(0, weight=1)

    def create_control_frame(self):
        self.control_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.control_frame.grid(row=1, column=0, sticky="nsew")
        num_actions = len(possible_actions)

        action_selectors = []
        self.act_variables = []
        for a in range(num_actions):
            self.act_variables.append(Tk.StringVar(self.control_frame))
            self.act_variables[a].set(possible_actions[a])  # default value

            action_selectors.append(Tk.OptionMenu(self.control_frame, self.act_variables[a], *possible_actions))
            action_selectors[-1].grid(row=a, column=0, columnspan=2, sticky="nsew")
            self.control_frame.grid_rowconfigure(a, weight=1)

        # Start task button
        self.start_button = Tk.Button(master=self.control_frame, text="Start", command=self._start, bg="green", padx=50, pady=20)
        self.start_button.grid(row=len(action_selectors), column=0, sticky="nsew")

        # Quit button
        self.quit_button = Tk.Button(master=self.control_frame, text="Quit", command=self._quit, bg="red", padx=50, pady=20)
        self.quit_button.grid(row=len(action_selectors), column=1, sticky="nsew")

        # Adjust spacing of objects
        self.control_frame.grid_columnconfigure(0, weight=1)
        self.control_frame.grid_columnconfigure(1, weight=1)

    def create_localisation_frame(self):
        self.localisation_frame = Tk.Frame(master=self.root, bg="dodger blue")
        self.localisation_frame.grid(row=1, column=1, sticky="nsew")

        # Tasks List
        self.load_localisations()
        self.localisations = ttk.Treeview(self.localisation_frame, show=[
                                  "headings"], height=len(self.localisations_data.index), displaycolumns="#all")
        self.localisations.grid(row=0, column=0, sticky='nsew')
        self.localisations["columns"] = self.col_names

        for i in self.col_names:
            self.localisations.column(i, anchor="center", stretch=True, width=20)
            self.localisations.heading(i, text=i, anchor='center')

        for index, row in self.localisations_data.iterrows():
            self.localisations.insert("", index=index, values=list(row))

    def load_localisations(self):
        self.col_names, localisation_list = self.db.query_table('detected_objects', 'all')
        self.localisations_data = pd.DataFrame(localisation_list, columns=self.col_names)

    def _quit(self):
        global QUIT
        QUIT = True
        self.root.quit()     # stops mainloop
        self.root.destroy()  # this is necessary on Windows to prevent

    def _start(self):
        actionlist = []
        for a in self.act_variables:
            actionlist.append(a.get())
        print(f"Ready to start task: {actionlist}")

    def load_robot_actions_data(self):
        self.col_names, actions_list = self.db.query_table('robot_action_timings', 'all')
        self.robot_tasks_data = pd.DataFrame(actions_list, columns=self.col_names)
        for row in self.robot_tasks_data.itertuples():
            self.robot_tasks_data.at[row.Index, 'robot_start_t'] = max(0.0, round(row.robot_start_t.total_seconds(), 2))

    def update_gui(self):
        # Update gui
        #self.root.update_idletasks()
        self.root.update()


def run_gui():
    # Run ROS node
    frame_id = 'gui_node'
    # rospy.init_node(frame_id, anonymous=True)

    cmd_publisher = None#rospy.Publisher('ProcessCommands', String, queue_size=10)

    gui = GUI(cmd_publisher)

    while True:#not rospy.is_shutdown():
        if QUIT:
            break
        else:
            try:
                gui.update_gui()
            except Exception as e:
                print(f"Error: {e}")
                break


if __name__ == '__main__':
    # Run GUI
    # try:
        run_gui()
    # except rospy.ROSInterruptException:
        # pass

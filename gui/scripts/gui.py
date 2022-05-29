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
from tkinter import CENTER, ttk
import argparse
import threading
from database_funcs import database
import rospy
from std_msgs.msg import String, Bool
os.chdir(os.path.dirname(os.path.realpath(__file__)))

plt.ion()

QUIT = False
possible_actions = ["Look for objects", "Red button", "Blue button", "key", "AA batteries", "Ethernet", "Cell battery", "None"]


class GUI:
    def __init__(self, cmd_publisher, local_pub):
        self.db = database()
        # Create GUI
        self.root = Tk.Tk()
        self.root.wm_title("Robothon interaction screen")
        self.root.resizable(True, True)
        self.cmd_publisher = cmd_publisher
        self.local_pub = local_pub

        self.create_header_frame()
        self.create_control_frame()
        self.create_localisation_frame()

        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=1)

        self.localisationInd = None

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
        self.team_text = Tk.Label(master=self.header_frame, text="Team inte-R-action", bg="dodger blue", font="none 20 bold")
        self.team_text.config(anchor=CENTER)
        self.team_text.grid(row=0, column=0, sticky="nsew")

        # Competition text
        self.comp_text = Tk.Label(master=self.header_frame, text="Robothon 2022", bg="dodger blue", font="none 20 bold")
        self.comp_text.config(anchor=CENTER)
        self.comp_text.grid(row=0, column=2, sticky="nsew")

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
                                  "headings"], height=13, displaycolumns="#all")
        self.localisations.grid(row=0, column=0, columnspan=2, sticky='nsew')
        self.localisations["columns"] = self.col_names

        for i in self.col_names:
            self.localisations.column(i, anchor="center", stretch=True, width=70)
            self.localisations.heading(i, text=i, anchor='center')

        for index, row in self.localisations_data.iterrows():
            self.localisations.insert("", index=index, values=list(row))
        
        self.localActive_label = Tk.Label(master=self.localisation_frame, bg="yellow", text="Localisation Active",
                                   padx=10, pady=3, borderwidth=2)
        self.localActive_label.grid(row=1, column=0, columnspan=2, sticky="nsew")
        
        # Start localisation button
        self.start_local_button = Tk.Button(master=self.localisation_frame, text="Start Localisation", command=self._startLocal, bg="green", padx=50, pady=20)
        self.start_local_button.grid(row=2, column=0, sticky="nsew")

        # Stop localisation button
        self.stop_local_button = Tk.Button(master=self.localisation_frame, text="Stop Localisation", command=self._stopLocal, bg="red", padx=50, pady=20)
        self.stop_local_button.grid(row=2, column=1, sticky="nsew")

        # Adjust spacing of objects
        self.localisation_frame.grid_rowconfigure(0, weight=1)
        self.localisation_frame.grid_rowconfigure(1, weight=0)
        self.localisation_frame.grid_columnconfigure(0, weight=1)
        self.localisation_frame.grid_columnconfigure(1, weight=1)

    def load_localisations(self):
        try:
            self.col_names, localisation_list = self.db.query_table('detected_objects', 'all')
            self.localisations_data = pd.DataFrame(localisation_list, columns=self.col_names)
            for row in self.localisations_data.itertuples():
                self.localisations_data.at[row.Index, 'confidence'] = round(row.confidence, 2)
                self.localisations_data.at[row.Index, 'rotation'] = round(row.rotation, 2)
                self.localisations_data.at[row.Index, 'center_x'] = round(row.center_x, 2)
                self.localisations_data.at[row.Index, 'center_y'] = round(row.center_y, 2)
                self.localisations_data.at[row.Index, 'distance'] = round(row.distance, 2)
        except Exception as e:
            print(e)

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
    
    def _startLocal(self):
        self.local_pub.publish(True)

    def _stopLocal(self):
        self.local_pub.publish(False)
    
    def update_localisationInd(self, data):
        self.localisationInd = data.data

    def update_gui(self):
        self.load_localisations()
        self.localisations.delete(*self.localisations.get_children())
        for index, row in self.localisations_data.iterrows():
            self.localisations.insert("", index=index, values=list(row))
        
        if self.localisationInd is None:
            self.localActive_label.config(bg='yellow')
        elif self.localisationInd:
            self.localActive_label.config(bg='green')
        else:
            self.localActive_label.config(bg='red')

        # Update gui
        #self.root.update_idletasks()
        self.root.update()


def run_gui():
    # Run ROS node
    frame_id = 'gui_node'
    rospy.init_node(frame_id, anonymous=True)
    
    cmd_publisher = rospy.Publisher('ProcessCommands', String, queue_size=10)
    local_pub = rospy.Publisher('UpdateLocations', Bool, queue_size=10)
    
    gui = GUI(cmd_publisher, local_pub)

    rospy.Subscriber('LocalisationActive', Bool, gui.update_localisationInd)

    while not rospy.is_shutdown():
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
    try:
        run_gui()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3.7
# Should be run from multimodal_human_robot_collaboration folder

from fileinput import filename
import sys, os
import rospy
import argparse
import traceback
from database_funcs import database
from os import listdir
from os.path import isfile, join
import csv
import pandas as pd
from statistics import mean

os.chdir(os.path.expanduser("~/catkin_ws/src/robothon/"))

#Define tables: tables = [{name, [col1 cmd, col2 cmd, ...]}, ]
tables_to_make = ['detected_objects']
tables = [['detected_objects', ["identification SERIAL PRIMARY KEY",
                                "obj_id INTEGER",
                                "obj_name VARCHAR(255) NOT NULL",
                                "confidence FLOAT",
                                "rotation FLOAT",
                                "center_x FLOAT",
                                "center_y FLOAT",
                                "distance FLOAT"]]]

def sys_stat_callback(data):
    """callback for system status messages"""
    if data.Header.frame_id == 'gui_node':
        if data.DiagnosticStatus.message == 'SHUTDOWN':
            rospy.signal_shutdown('gui shutdown')


def make_tables(db, del_tab=True):
    try:
        table_avail = [item[0] for item in tables]
        assert all(elem in table_avail for elem in tables_to_make), "Some tables to make not in tables list"
        curr_tables = db.table_list()
        print(f"Tables to create: {tables_to_make}")

        for name in tables_to_make:
            if (name in curr_tables) and not del_tab:
                print(f"Table '{name}' already exists, leaving as is")
            else:
                if name in curr_tables:#del_tab:
                    print(f"Table '{name}' alredy exists, deleting")
                    db.remove_table(name)
                _, cmd = [i for i in tables if i[0] == name][0]
                db.create_table(name, cmd)
                print(f"Successfully created table '{name}'")

    except AssertionError as e:
        print(f"Assertion Error: {e}")
        raise
    except Exception as e:
        print(f"Make Tables Error: {e}")
        raise


# def load_tables(db):
#     base_dir = os.getcwd()+'/sam_nodes/scripts/postgresql/'
#     for name in tables_to_make:
#         try:
#             db.csv_import(f"{base_dir}{name}.csv", tab_name=name)
#             if name in ('assemble_box', 'assemble_chair', 'stack_tower', 'assemble_complex_box', 'assemble_complex_box_manual'):
#                 # Update times and action ids from actions table
#                 sql = f"UPDATE {name} SET action_id = actions.action_id, default_time = actions.std_dur_s FROM actions WHERE actions.action_name = {name}.action_name"
#                 db.gen_cmd(sql)

#             print(f"Loaded data into '{name}'")

#         except FileNotFoundError:
#             print(f"WARNING: Load table file not found for '{name}' at {base_dir}{name}.csv")
#         except Exception as e:
#             print(f"Load Table Error: {e}")
#             raise
#     print("Load Tables Completed")


def save_tables(db, tables_to_save='all', file_path=None, verbose=True):
    if tables_to_save == 'all':
        tables_to_save = db.table_list(verbose=False)
    
    for table in tables_to_save:
        try:
            db.csv_export(table, file_path=f"{file_path}/{table}.csv", verbose=True)
        except Exception as e:
            if verbose:
                print(e)
            raise


def shutdown(db):
    #always save tables to dump on exit
    try:
        save_tables(db, tables_to_save='all', file_path=os.path.dirname(__file__)+'/postgresql/dump', verbose=False)
    except Exception as e:
        print(f"Dump tables error: {e}")
        raise

    print("Database node shutdown")


def database_run(db):
    # ROS node setup
    frame_id = 'Database_node'
    rospy.init_node(frame_id, anonymous=True)

    rate = rospy.Rate(1) # 1hz
    try:
        # Test connection
        db.connect(verbose=True)
        db.disconnect(verbose=True)
        # Make and load predefined tables
        make_tables(db)
        # load_tables(db)
    except Exception as e:
        print(f"Database node create database error: {e}")
        raise

    while not rospy.is_shutdown():
        try:
            # Test database connection to ensure running smoothly
            db.connect()
            db.disconnect()
        except Exception as e:
            print(f"Database connection error: {e}")
        rate.sleep()


if __name__ == '__main__':
    db = None
    try:
        db = database()
        database_run(db)
    except rospy.ROSInterruptException:
        print("database_run ROS exception")
    except Exception as e:
        print("**Database Error**")
        traceback.print_exc(file=sys.stdout)
    finally:
        if db is not None:
            shutdown(db)
        pass

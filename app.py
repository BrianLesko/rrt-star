##################################################################
# Brian Lesko 
# 12/2/2023
# Robotics Studies, RRT Search
# Rapidly exploring Random Tree

import streamlit as st
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt

import customize_gui # streamlit GUI modifications
import robot
import rrt as RRT
gui = customize_gui.gui()
my_robot = robot.two2_robot()
Obstacles = robot.TwoD_objects()
rrt = RRT.RRT([0, 0], [1, 1])

def main():
    # Set up the app UI
    gui.clean_format(wide=True)
    Sidebar = gui.about(text = "This code implements the RRT algorithm for robotic path planning.")
    Title, subTitle, image_spot = st.empty(), st.sidebar.empty(), st.columns([1,5,1])[1].empty()
    
    with Sidebar: st.button("Reset")

    # Initialize loop variables
    thetas = [0,0]
    fig, ax = my_robot.get_colored_plt("#F6F6F3",'#335095','#D6D6D6')
    my_robot.set_c_space_ax(ax)

    # Determine which joint is selected
    joints_label = "<span style='font-size:30px;'>RRT:</span>"
    j1 = "<span style='font-size:20px;'>Random Sampling</span>"
    with Title: st.markdown(f" {joints_label} &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;{j1} &nbsp; &nbsp; &nbsp; ", unsafe_allow_html=True)

    #RRT Loop
    for i in range(555):
        x_samp = rrt.get_x_sample()
        x_nearest = rrt.nodes[rrt.get_nearest_node(x_samp)]
        x_new = rrt.get_x_new(x_nearest, x_samp)
        ax.plot([x_nearest[0], x_new[0]], [x_nearest[1], x_new[1]], color="#335095", linewidth=2, solid_capstyle='round')
        rrt.check_collision(x_new)
        if i % max(2, i // 10) == 0: 
            with image_spot: st.pyplot(fig)
    with image_spot: st.pyplot(fig)  # Display the final result

main()
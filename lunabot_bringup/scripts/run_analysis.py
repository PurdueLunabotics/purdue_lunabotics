# Made by Peter Timpane 2023

import matplotlib.pyplot as plt
import rosbag
from moviepy.editor import VideoFileClip

# Dictionaries for the current and effort data
cur = {}
eff = {}

# Get the inputs from the user
print("Enter bag file name (include .bag)")
bag_name = input()
print("Enter video file name (include extension)")
vid_name = input()
print("Enter start time of video for data in seconds")
data_start = int(input())
print("Enter the time to be examined in seconds (or -1 for the entire graph)")
time_chosen = int(input())
if time_chosen != -1:
    print("How many seconds on either end of the chosen time?")
    buffer = int(input())


try:
    # Open the bag and get the state and effort data
    with rosbag.Bag(bag_name) as bag:
        for topic, msg, t in bag.read_messages(topics=["/state", "/effort"]):
            time = t.secs

            if topic == "/state":
                # List comprehension to get the current data from the message
                currents = [int(x.split(":")[1][1:]) for x in str(msg).split("\n")[0:6]]
                cur[time] = currents
            else:
                # List comprehension to get the effor data from the message
                efforts = [int(x.split(":")[1][1:]) for x in str(msg).split("\n")[6:]]
                eff[time] = efforts

    # Sort the data by chronological order
    cur_times = list(cur.keys())
    cur_times.sort()
    # Make it so 0 time is the start of the video
    cur = {x - cur_times[0] - data_start: cur[x] for x in cur_times}

    eff_times = list(eff.keys())
    eff_times.sort()
    eff = {x - eff_times[0] - data_start: eff[x] for x in eff_times}

    # Remove all the data before the start of the video
    i = 0
    for x in cur.keys():
        if x < 0:
            i += 1
        else:
            break

    cur = {x: cur[x] for x in list(cur.keys())[i:]}
    eff = {x: eff[x] for x in list(eff.keys())[i:]}

    # create the subplot
    fig, ax = plt.subplots(nrows=2)

    ax[1].set_ylim(-135, 135)

    # User specified a time
    if time_chosen != -1:
        ax[0].plot(
            [
                x - time_chosen + buffer
                for x in list(cur.keys())[time_chosen - buffer : time_chosen + buffer]
            ],
            list(cur.values())[time_chosen - buffer : time_chosen + buffer],
            linewidth=1.0,
        )
        ax[0].legend(
            [
                "Lead Screw",
                "Actuation",
                "Depositon",
                "Excavation",
                "Drive Left",
                "Drive Right",
            ]
        )
        ax[0].title.set_text("Current Data")
        ax[1].plot(
            [
                x - time_chosen + buffer
                for x in list(eff.keys())[time_chosen - buffer : time_chosen + buffer]
            ],
            list(eff.values())[time_chosen - buffer : time_chosen + buffer],
            linewidth=1.0,
        )
        ax[1].legend(
            [
                "Lead Screw",
                "Actuation",
                "LeftDrive",
                "Right Drive",
                "Excavate",
                "Deposit",
            ]
        )
        ax[1].title.set_text("Effort Data")

        # Output the video clip
        video = VideoFileClip(vid_name)
        clip = video.subclip(time_chosen - buffer, time_chosen + buffer)
        clip.write_videofile("video_clip.mp4")

    else:  # No time specified, no clip
        ax[0].plot(cur.keys(), cur.values(), linewidth=1.0)
        ax[0].legend(
            [
                "Lead Screw",
                "Actuation",
                "Depositon",
                "Excavation",
                "Drive Left",
                "Drive Right",
            ]
        )
        ax[0].title.set_text("Current Data")
        ax[1].plot(eff.keys(), eff.values(), linewidth=1.0)
        ax[1].legend(
            [
                "Lead Screw",
                "Actuation",
                "LeftDrive",
                "Right Drive",
                "Excavate",
                "Deposit",
            ]
        )
        ax[1].title.set_text("Effort Data")

    # Output the plot
    plt.savefig("graph_clip")

except FileNotFoundError:
    print("ERROR: File not found")

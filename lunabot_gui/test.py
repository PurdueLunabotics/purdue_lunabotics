import subprocess
import threading
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
import paramiko

class ROSLauncherApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Launcher")

        self.processes = {}  # Store processes by name
        self.text_widgets = {}  # Store text widgets by name

        self.control_frame = tk.Frame(root)
        self.control_frame.pack(pady=5)

        self.process_frame = tk.Frame(root)
        self.process_frame.pack(pady=5)

        self.ssh_client = None  # SSH client for remote connection

        # Add two processes by default
        self.add_new_process("Computer", "lunabot_bringup", "computer.launch")
        self.add_new_process("Process 2", "package2", "launch2.launch")

        self.add_remote_process("Remote Process")


    def stream_output(self, process, text_widget):
        """Streams combined stdout and stderr to a text widget in real time."""
        for line in iter(process.stdout.readline, ''):
            text_widget.insert(tk.END, line)
            text_widget.see(tk.END)  # Auto-scroll to latest output
        process.stdout.close()

    def start_process(self, name, package, launch_file):
        """Starts a ROS launch process with given parameters."""
        if name in self.processes:  # Prevent duplicate processes
            return

        process = subprocess.Popen(
            ['roslaunch', package, launch_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True
        )

        self.processes[name] = process
        threading.Thread(target=self.stream_output, args=(process, self.text_widgets[name]), daemon=True).start()


    def add_new_process(self, name=None, package="default_package", launch_file="default.launch"):
        """Adds a new text box and button for starting a new ROS launch process."""
        if name is None:
            name = f"Process {len(self.processes) + 1}"

        # Create a new frame for the process
        frame = tk.Frame(self.process_frame)
        frame.pack(pady=5, fill="x")

        # Create a text widget for the process
        text_widget = ScrolledText(frame, height=10, width=60)
        text_widget.pack(side="left", padx=5)

        self.text_widgets[name] = text_widget

        button = tk.Button(frame, text=f"Start {name}",
                           command=lambda: self.start_process(name, package, launch_file))
        button.pack(side="right", padx=5)


    def add_remote_process(self, name):
        """Adds an entry for the remote ROS launch process."""
        frame = tk.Frame(self.process_frame)
        frame.pack(pady=5, fill="x")

        text_widget = ScrolledText(frame, height=10, width=60)
        text_widget.pack(side="left", padx=5)

        self.text_widgets[name] = text_widget

        # SSH Connect Button
        connect_button = tk.Button(frame, text="Connect to Remote", command=self.connect_ssh)
        connect_button.pack(side="right", padx=5)

        # Remote Start Button (Disabled until SSH connects)
        self.remote_start_button = tk.Button(frame, text="Start Remote Process", state=tk.DISABLED, 
                                             command=lambda: self.start_remote_process(name, "remote_package", "remote.launch"))
        self.remote_start_button.pack(side="right", padx=5)

    def connect_ssh(self):
        """Establishes SSH connection to the remote machine."""
        host = "192.168.0.111"  # Change to your remote IP
        username = "robot"
        password = "goboilers!!"  # Prefer SSH keys instead of a password

        try:
            self.ssh_client = paramiko.SSHClient()
            self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            print("try connect")
            self.ssh_client.connect(host, username=username, password=password)
            print("done")
            self.text_widgets["Remote Process"].insert(tk.END, "SSH Connection Successful\n")
            self.remote_start_button.config(state=tk.NORMAL)  # Enable remote launch button
        except Exception as e:
            self.text_widgets["Remote Process"].insert(tk.END, f"SSH Connection Failed: {e}\n")

    def start_remote_process(self, name, package, launch_file):
        """Starts a ROS launch process on the remote machine via SSH."""
        if not self.ssh_client:
            self.text_widgets[name].insert(tk.END, "Not connected to remote machine\n")
            return

        command = f"roslaunch {package} {launch_file}"

        try:
            stdin, stdout, stderr = self.ssh_client.exec_command(command)

            def read_remote_output(stream):
                for line in iter(stream.readline, ''):
                    self.text_widgets[name].insert(tk.END, line)
                    self.text_widgets[name].see(tk.END)

            # Stream output in real time
            threading.Thread(target=read_remote_output, args=(stdout,), daemon=True).start()
            threading.Thread(target=read_remote_output, args=(stderr,), daemon=True).start()

        except Exception as e:
            self.text_widgets[name].insert(tk.END, f"Failed to start remote process: {e}\n")



if __name__ == "__main__":
    root = tk.Tk()
    app = ROSLauncherApp(root)
    root.mainloop()

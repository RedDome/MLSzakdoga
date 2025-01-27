import subprocess
import time

# Path to your .sh file
sh_file_path = './start_ros.sh'

# Run the shell script in the background
# subprocess.Popen(['bash', sh_file_path])

# Wait a moment to ensure the shell script has started (optional, depending on your use case)
# time.sleep(5)

print("Sleep completed")

# Call another Python script
subprocess.run(['python3', 'src/main.py'])

print("Shell script is running in the background, and the second Python script has been called.")

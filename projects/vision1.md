---
layout: work
type: Project
num: 6
worktitle: Action Selection with Images
---

## Sample Program
* [`main.py`]({{site.baseurl}}/assets/programs/vision1/main.py)
* [`lib.py`]({{site.baseurl}}/assets/programs/vision1/lib.py)

## Network Communication
The robot and tablet will communicate using the 
[`HendrixIOT`](https://www.hendrix.edu/HelpDesk/Wifi_and_Networking/Wireless/Connecting_to_the_HendrixIOT_Network/) 
wifi network. Following the 
[instructions](https://www.hendrix.edu/HelpDesk/Wifi_and_Networking/Wireless/Connecting_to_the_HendrixIOT_Network/),
connect your tablet to that network.

The robot and tablet communicate using a [`socket`](https://docs.python.org/3/library/socket.html).
The code example below shows how we can communicate over a socket:
```
import socket

SERVER_IP = "172.17.3.91"
PORT = 8888

def send_message(message):
    reply = None
    try:
        sock = socket.socket()
        sock.connect((SERVER_IP, PORT))
        sock.send(message.encode())
        reply = sock.recv(1024).decode()
    except Exception as e:
        reply = str(e)
    finally:
        sock.close()
    return reply
```

## Requesting Image Classification

To set up a classifier, send the app the following message:
* `knn [k] [project_name]`

For example, if you wish to start classifying with $k = 3$ and a project
you created called `avoid`, you would send:
* `knn 3 avoid`

To ask for the most recently assigned label for an image:
* `classify`

## Part 1: Obstacle Avoidance
* Modify your robot so that it can carry your tablet.
* Create an obstacle avoider that works entirely by use of images.
* To this end, create a `project` in the `VisionBot` app with two labels: 
  `obstacle` and `clear`. 
* Take a variety of representative photographs of each label. I recommend
  taking the photographs while the tablet is in its position on the robot.
* Use the sample program provided to test your classifier.
* Experiment with different sets of photographs until you achieve a 
  successful obstacle avoider.
  
## Part 2: Your Choice
* Devise **two** additional tasks using the `VisionBot` app.
* At least one of the tasks should employ three distinct labels.
* At least one of the tasks should incorporate ideas from either 
  of the [Mode Selection](https://hendrix-cs.github.io/csci235/projects/modes.html)
  or [Reinforcement Learning](https://hendrix-cs.github.io/csci235/projects/qlearning.html)
  projects.
  
## Questions
1. How did the purely-visual obstacle avoider perform in comparison
   to the obstacle avoiders that used sonars and bump sensors? 
   Describe specific observations to justify your answer.
2. What were the goals for your **first** additional task? How did the
   robot perform with respect to those goals?
3. What were the goals for your **second** additional task? How did the
   robot perform with respect to those goals?
4. How many distinct images per label did you need for each of your tasks
   for the robot to perform adequately?
5. What advantages and disadvantages did you find with using image
   processing as compared to the other sensors we have employed so far?


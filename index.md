---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: course-single
---

# <a name="description">Overview</a>

<p class="course-desc">
{{ site.description }}
</p>

## <a name="goals">Learning Goals</a>

Upon completing this course, our goal is for you to be able to:
* Program a mobile robot to interact with a realistic environment with natural timing.
* Employ classes and modules to structure Python programs.
* Create ROS2 nodes that: 
  * Control motors and process sensor information by publishing and subscribing to topics.
  * Create new topics and publish to them to enable building robust, modular controllers.
* Quantitatively and qualitatively assess the performance of a mobile robot for a given task.
* Employ computer vision techniques to inform the behavior of a mobile robot.
* Employ appropriate machine learning algorithms to improve the performance of a mobile robot.
* Employ planning and odometry to navigate a mobile robot.


## <a name="resources">Resources</a>

{% include resources.html content=site.resources %}

# <a name="calendar">Calendar</a>

Open office hours (in addition to [appointments](https://drferrer.youcanbook.me/)):
* Wednesdays 4:10-4:45
  * Except September 9, September 30, November 11, November 25, December 2
* Anytime my door is open

|Date   |Day  | Module | Due |
| ---   | --- | ------ | --- | 
|  8/25 | Tue | Module 1: Python classes |  |
|  8/27 | Thu | Module 2: Curses | Module 1 |
|       |
|  9/1  | Tue | Module 3: ROS2 nodes, Motor Publications | Module 2 | 
|  9/3  | Thu | Module 4: Subscriptions | Module 3 |
|       |
|  9/8  | Tue | Project 1 | Module 4 |
|  9/10 | Thu | Project 1 | Interview 1: Modules 1-4 (Fri 9/11) |
|       |
|  9/15 | Tue | Project 1 Presentations | Project 1 |  
|  9/17 | Thu | Module 5: State Machines |  |  
|       |
|  9/22 | Tue | Module 6: Fuzzy Logic | Module 5 | 
|  9/24 | Thu | Module 7: Computer Vision 1 | Module 6<br>Interview 2: Project 1 (Fri 9/25) | 
|       |
|  9/29 | Tue | **Conference: No class<br>Work on Project 2** | Module 7 | 
| 10/1  | Thu | **Conference: No class<br>Work on Project 2** | |
|       |
| 10/6  | Tue | Project 2 Presentations | Project 2 | 
| 10/8  | Thu | Module 8: SLAM 1 | Interview 3: Modules 5-7 (Fri 10/9) |
|       |
| 10/13 | Tue | Module 9: SLAM 2 | Module 8 |
| 10/15 | Thu | **Fall Break: No class** |  |   
|       |
| 10/20 | Tue | Module 10: Path Planning | Module 9 |   
| 10/22 | Thu | Module 11: HTN Planning | Module 10<br>Interview 4: Project 2 (Fri 10/23) | 
|       |
| 10/27 | Tue | Project 3 | Module 11 |  
| 10/29 | Thu | Project 3 | Interview 5: Modules 8-11 (Mon 11/2)|  
|       |
| 11/3  | Tue | Project 3 Presentations | Project 3 |
| 11/5  | Thu | [Module 12: Reinforcement Learning]({{site.baseurl}}/modules/qlearning.html) | Interview 6: Project 3 (Mon 11/9) |
|      
| 11/10 | Tue | Module 13: Computer Vision 2  | Module 12 |   
| 11/12 | Thu | Module 14: Computer Vision 3  | Module 13 |
|       |
| 11/17 | Tue | [Final Project]({{site.baseurl}}/projects/Final.html) | Module 14 |
| 11/19 | Thu | Final Project proposal presentations | Final Project Proposal<br>Interview 7: Modules 12-14 (Fri 11/20) |  
|       |
| 11/24 | Tue | Final Project work day (**optional**) |  |  
| 11/26 | Thu | **Thanksgiving: No class** |  |  
|       |
| 12/1  | Tue | Final Project work |  |  |  
| 12/3  | Thu | Final Project work |  |  |  
|       |
| 12/8  | Tue 8:30-11:30 am | Final Project Presentations | | [Final Projects]({{site.baseurl}}/projects/Final.html)<br>Interview 8: Final Project (Mon 12/14) |

# <a name="integrity">Academic Integricy</a>

Hendrix College is committed to high standards of honesty and fairness in academic pursuits. Such standards are central to the process of intellectual inquiry, the development of character, and the preservation of the integrity of the community.

This commitment can be understood in the following ways:

* A student should not gain an unfair advantage or violate other students’ commitment to honest work and genuine effort.  The amount of cooperation undertaken with other students, the consistency and accuracy of work, and the test-taking procedure should adhere to the guidelines that the classroom teacher provides.
* Members of the Hendrix community value and uphold academic integrity because we recognize that scholarly pursuits are aimed at increasing the shared body of knowledge.  The full disclosure of sources is the most effective way to ensure accountability to both ourselves and our colleagues.

More details can be found in the [Hendrix Academic Integrity Policy](https://www.hendrix.edu/Catalog/2026-2027/Academic_Policies_and_Regulations/Policies_and_Appeals/D_6_c__Academic_Integrity/) and the [Hendrix
Computer Science Academic Integrity Policy](http://ozark.hendrix.edu/~yorgey/ac-integrity-policy.html).

# <a name="ai">AI Policy</a>

Learning to write computer programs doesn't only develop programming skill - it also 
develops broader skills in formal reasoning. Using AI coding assistants disrupts the
development of these essential formal reasoning skills. As one of the goals of this course
is for you to further develop your programming and formal reasoning skills, the use of
AI coding assistants is prohibited for all programming assignments submitted for credit 
in this course.

Learning to write prose is also an essential skill that we seek to develop in this course
through writing reflections on modules and projects. Your prose need not be perfect - I 
want to read your own authentic voice, not that of an AI writing assistant. I want you to
develop your own authentic voice, not that of a machine. Thus, the use of generative AI
writing assistants is also prohibited for all writing assignments submitted for credit
in this course.

# <a name="assessment">Assessment</a>

## <a name="journal">Journal</a>

Every student is expected to maintain a journal as a Word document stored in Microsoft Teams. 
Each student has a private channel in Teams, in which their journal document is stored. For
every working session, the student should record the following in their journal:
* Time when work started
* Goals for the work session
* Discussion of the robot programming process
  * What was your approach to writing code to achieve the session goals?
  * What went well? 
  * What were some challenges you overcame?
* Records of observations
  * How did the robot perform for the session's tasks?
  * Record any quantitative data to support your claims about its performance.
* Time when work ended, including total duration of the work session 
* Journal entries are assessed as part of assessing each module and project. Journal
  entries may also serve as discussion items during interviews.

## <a name="attendance">Attendance</a>
Most of our work in this course requires **teamwork**. Furthermore, robotics is a 
challenging topic - success as a roboticist requires consistent engagement. For these 
reasons, class attendance is **mandatory** on all class days, except those marked 
**optional** or **no class**. If a student misses class, the following steps are required 
to make up the credit:
* Notify the instructor of the reason for the absence by 5 pm on the day of the missed 
class.
* Speak to at least one other student in the class about the content of the class period.
* Write a summary of the missed class in your journal before the start of the next class
  period. This summary should mention the students with whom you spoke about the missed 
  class. Discussion of the missed class should include not only the lecture material but
  key lessons learned from that day's module or presentations.
* This also applies to any student who adds the class after the first day - those prior days
  should be made up in this same fashion.

Including the final exam period, there are 24 required class meetings, from which students
may earn up to 24 **participation credits**.

## <a name="modules">Modules</a>
The primary means by which you be introduced to the course material is through 14 
**modules**. Each module will be started during class time in teams of 2 or 3. Each module 
will guide you through a learning process on a particular topic. Modules not completed during 
class time should be completed with your group outside of class. 

Module submissions consist of two parts:
* Each module contains questions expecting short answers. Those questions and answers should
  be included in your journal.
* Python programs to modify or write. These should be submitted via Teams **individually** by
  each student. As the modules are a key aspect of how students learn the course material, this ensures that
every student has a record of what was learned.

Modules submitted by the start of the next class period will receive one robot credit for a
sincere, on-time attempt. Submissions which are complete and correct will receive two
robot credits. Modules not deemed complete and correct may be revised and resubmitted after a 
conversation with the professor to address the issues of concern, whereupon they will
receive the second robot credit. Modules submitted for the first time after the deadline
will receive at most one robot credit.

## <a name="projects">Projects</a>
The modules are grouped into four **units**. At the completion of each of the first three units,
I will assign a project. In each project, students will devise a creative robotics project 
applying the ideas investigated in the just-completed unit.

The first class day for each project will be a brainstorming session in which students devise
topics, select project partners, establish project goals, and begin work on the project. 
The second class day will be a working session for the project, including a brief oral 
presentation of current project progress. On the third class day, each group will give 
a presentation to the class about their project.

### Project Reports
For each project, each student (even if part of a team) should write a project
report entry in their journal. Each project report entry includes the following:
* Description of the project goals
* A discussion of each ROS2 node created for the project.
  * For each node, discuss the following:
    * To what does it subscribe?
    * To where does it publish?
    * How does it determine what to publish?
    * What other interactions with the environment does it have?
* A discussion of how the ROS2 nodes worked together to enable
  the robot to achieve the project goals.
* A conclusion detailing the degree of success of the project and
  describing future work, that is, the next steps to take if the
  project were to continue.

### Project Presentations

On project work days prior to the due date of each project, each team will give a 
brief project update. The project update should include three slides:
* A title slide, including the name of the project and team participants.
* A slide describing the project goals.
* A slide discussing the current status of the project.

On the due date of each project, each team will give a presentation about their project.
The presentation should include the following:
* Five to seven slides:
  * A title slide, including the name of the project and team participants.
  * A slide describing the project goals.
  * 1-3 slides describing the ROS2 nodes created for the project.
    * For each node, discuss the following:
      * To what does it subscribe?
      * To where does it publish?
      * How does it determine what to publish?
      * What other interactions with the environment does it have?
  * An assessment of the degree to which project goals were met.
  * Future work
    * If one were to continue the project, what would be the next things to address?
* A video of the robot in action. The video should meet the following constraints:
  * It must be between 60 and 120 seconds in duration.
  * It should include brief narration of the strategy for the project.
    * Narration may be pre-recorded or provided live as the video runs.
  * It should demonstrate the student's robots fulfilling the goals for the project. 
    * Narration should contextualize each demonstrated activity.
  
### Presentation Questions
* Students should be prepared to answer questions after their video presentation concludes. 
* Each student is expected to ask one substantive question on each class day that includes presentations.
  
### Project Credits
Robot credits for projects will be awarded as follows:
* For each project (not including the final project), you may earn up to 5 robot credits:
    * Achievement of project goals
      * Achieving all agreed-upon project goals: **2 robot credits**
        * **Note**: If, as the project develops, a project goal appears unrealistic to achieve, the students
          and professor may renegotiate the project goals **at least one day prior to the submission deadline**.
      * Achieving some but not all of the agreed-upon project goals: **1 robot credit**
    * Submitting a satsifactory project report: **1 robot credit**
    * Delivering a satisfactory project presentation: **1 robot credit**
    * Submitting the project code and report on-time, and delivering the presentation on the assigned day: **1 robot credit**
* Asking questions during presentations:
  * Ask at least one question on each of the four presentation days: **2 robot credits**
  * Ask at least one question on each of two presentation days: **1 robot credit**
  
## <a name="interviews">Interviews</a>
After each module unit and after each project, each student will 
[schedule an interview](https://drferrer.youcanbook.me/) to discuss the completed
work and demonstrate mastery of the concepts behind the completed work. 
* A Level 1 interview demonstrates some understanding of the topic but with significant gaps.
* A Level 2 assessment demonstrates full expected understanding of the topic.
 
### Interview Credits 
* One interview credit will be awarded per level.
* One additional interview credit will be awarded for completing the interview by the deadline.
* If a student is unsatisfied by their interview credit, they may schedule a follow-up 
  interview to demonstrate stronger understanding. 

## <a name="finalproject">Final Project</a>
In the last three weeks of the semester, each student will undertake a final project. 
In this final project, you will program a robot that fulfills a contextualized 
purpose. A demonstration will be made of the robot's capabilities, and a paper 
reflecting upon lessons learned will be submitted as well. In keeping with the Odyssey 
Special Project guidelines, the project will require at least 30 hours of work. As with 
the other course projects, final projects will be undertaken in teams of two or three.
The project report and presentation should follow the same guidelines as those for the
regular course projects.

The final project will be graded strictly on a pass-fail basis: it is either completed
or not completed, including the project report and presentation. It does not contribute
to robot credits or interview credits. As with the three regular course projects, if a 
project goal appears unrealistic to achieve, the students and professor may renegotiate 
the project goals **at least one day prior to the submission deadline**.
<hr>

## <a name="grading">Specifications Grading</a>
Final course grades are earned based on accumulated credits, as follows:

* To earn an A in the course, a student will:
  * Earn at least 42 out of 45 possible robot credits
  * Earn at least 22 out of 24 possible interview credits
  * Earn at least 23 out of 24 possible participation credits
  * Complete and present the final project
* To earn a B in the course, a student will: 
  * Earn at least 34 robot credits
  * Earn at least 17 interview credits
  * Earn at least 20 participation credits
  * Complete and present the final project
* To earn a C in the course, a student will:
  * Earn at least 26 robot credits
  * Earn at least 12 interview credits
  * Earn at least 16 participation credits
  * Complete and present the final project
* To earn a D in the course, a student will:
  * Earn at least 18 robot credits
  * Earn at least 7 interview credits
  * Earn at least 12 participation credits

## <a name="equipment">Equipment</a>
During the second week of the semester, each student team will be assigned
an iRobot Create3 robot and a Raspberry Pi controller for that robot. As 
teams shift over the course of the semester, robots will be reassigned to the 
new teams.

The instructor will handle configuration and setup, but for reference here
are the [Raspberry Pi 5 instructions]({{site.baseurl}}/robot_setup.html) and 
[Raspberry Pi 4 instructions]({{site.baseurl}}/robot_setup_pi_4.html).

The robots should be returned to the instructor at the end of the semester.
Students will be billed for any unreturned equipment.

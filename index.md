---
# Feel free to add content and custom Front Matter to this file.
# To modify the layout, see https://jekyllrb.com/docs/themes/#overriding-theme-defaults

layout: course-single
---

# <a name="description">Overview</a>

{{ site.description }}

## <a name="goals">Learning Goals</a>

Upon completing this course, our goal is for you to be able to:
* Program a mobile robot to interact with a realistic environment with natural timing.
* Create ROS2 nodes that: 
  * Control motors and process sensor information by publishing and subscribing to topics.
  * Create new topics and publish to them to enable building robust, modular controllers.
* Quantitatively and qualitatively assess the performance of a mobile robot for a given task.
* Describe both orally and in writing:
  * The design and performance of a mobile robot.
  * Assessments of deployed and fictional robots.
* Employ computer vision techniques to inform the behavior of a mobile robot.
* Employ appropriate machine learning algorithms to improve the performance of a mobile robot.
* Employ planning and odometry to navigate a mobile robot.

## <a name="resources">Resources</a>

{% include resources.html content=site.resources %}

<hr>

# <a name="calendar">Calendar</a>

|Date  |Day  |Topic/Module |Reading |Due |
| ---  | --- | ---         | ---    | --- |
| 1/21 | Tue | Overview<br>Robots in Science Fiction<br> [Robots: Fiction and Reality]({{site.baseurl}}/essays/robots_sci_fi.html) |  |
| 1/23 | Thu | Discussion of Short Stories | [Homework](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/philip-apps/homework)<br>[Company Property](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/john-albertson/company-property)<br>[Domotica Berserker](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/paul-g-di-filippo/domotica-berserker)<br>[30 Pounds of Human Tissue](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/jennifer-campbell-hicks/30-pounds-of-human-tissue)<br>[How I Saved the Galaxy on a Limited Budget](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/aidan-doyle/how-i-saved-the-galaxy-on-a-limited-budget)<br>[Memo From the Lab of the Moral Weapon](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/h-baumgardt/memo-from-the-lab-of-the-moral-weapon)<br>[Under My Thumb](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/laura-ansara/under-my-thumb) |  |
|      |
| 1/28 | Tue | [Module 1: ROS2 Nodes, Topics, and Subscriptions]({{site.baseurl}}/modules/nodes.html) | [A Robust Layered Control System for a Mobile Robot]({{site.baseurl}}/readings/Robust_Control.pdf) | Robots: Fiction and Reality |
| 1/30 | Thu | [Module 2: ROS2 Publications]({{site.baseurl}}/modules/publications.html) |  | Module 1 |
|      |
| 2/4  | Tue | [Module 3: State Machines]({{site.baseurl}}/modules/state_machines.html) | | Module 2 |
| 2/6  | Thu | Module 4: Fuzzy Logic | | Module 3 |
|      |
| 2/11 | Tue | Project 1 |  |  Module 4 |
| 2/13 | Thu | Project 1 |  |  |  
|      |
| 2/18 | Tue | Project 1 Presentations |  | Project 1 |
| 2/20 | Thu | In-Class Essay 1  |  |  |  
|      |
| 2/25 | Tue | Module 5: Reinforcement Learning  |  |  |
| 2/27 | Thu | Module 6: Vision 1: Motion |  | Module 5 |  
|      |
| 3/4  | Tue | Module 7: Vision 2: Recognition |  | Module 6 | 
| 3/6  | Thu | Project 2  |  | Module 7 |  
|      |
| 3/11 | Tue | Project 2 |  |  | 
| 3/13 | Thu | Project 2 Presentations |  | Project 2 |   
|      |
| 3/18 | Tue | In-Class Essay 2 |  |  |  
| 3/20 | Thu | Module 8: Planning |  |  | 
|      |
| 3/25 | Tue | Spring Break |  |  |  
| 3/27 | Thu | Spring Break |  |  |  
|      |
| 4/1  | Tue | Module 9: Mapping |  | Module 8 |  
| 4/3  | Thu | Project 3 |  | Module 9 |  
|      
| 4/8  | Tue | Project 3 |  |  |  
| 4/10 | Thu | Project 3 Presentations |  | Project 3 |  
|      |
| 4/15 | Tue | [Final Project]({{site.baseurl}}/projects/Final.html) |  |  
| 4/17 | Thu | Final Project work |  |  |
|      |
| 4/22 | Tue | Final Project proposal presentations |  | Final Project Proposal |
| 4/24 | Thu | In-Class Essay 3 |  |  |  
|      |
| 4/29 | Tue | Final Project Progress Reports |  |  |  
| 5/1  | Thu | Final Project work |  |  |  
|      |
| 5/6  | Tue 8:30-11:30 am | Final Project Presentations | | [Final Projects]({{site.baseurl}}/projects/Final.html) |

<hr>
# <a name="assessment">Assessment</a>

## <a name="modules">Modules</a>
The primary means by which you be introduced to the course material is through nine 
**modules**. Each module will be started during class time in teams of 2 or 3. Each module 
will guide you through a learning process on a particular topic. Modules not completed during 
class time should be completed with your group outside of class. 

Module submissions consist of two parts:
* Short answers to questions posed within the module.
* Python programs to modify or write.

Each student will submit both their answers and the Python programs **individually** via Teams.
As the modules are a key aspect of how students learn the course material, this ensures that
every student has a record of what was learned.

Modules submitted by the start of the next class period will receive one robot credit for a
sincere, on-time attempt. Submissions which are complete and correct will receive two
robot credits. Modules not deemed complete and correct may be revised and resubmitted after a 
conversation with the professor to address the issues of concern, whereupon they will
receive the second robot credit. Modules submitted for the first time after the deadline
will receive at most one robot credit.

## <a name="projects">Projects</a>
The modules are grouped into three **units**. At the completion of each unit, a project
will be assigned. In each project, students will devise a creative robotics project applying
the ideas investigated in the just-completed unit.

The first class day for each project will be a brainstorming session in which students devise
topics, select project partners, establish project goals, and begin work on the project. The 
second class day will be a working session for the project. On the third class day, each group 
will give a presentation to the class about their project.

### Project Reports
For each project, each student (even if part of a team) should submit an individual project
report. Each report includes the following:
* Description of the project goals
* A project log, which includes the following for every work session:
  * Date of the work session, including start and end times.
  * Goals for the session.
  * Brief descriptions of activities undertaken.
  * Observations of activities.
  * Assessment of the degree to which session goals were met.
* A discussion of each ROS2 node created for the project.
  * For each node, discuss the following:
    * To what does it subscribe?
    * To where does it publish?
    * How does it determine what to publish?
    * What other interactions with the environment does it have?
* A conclusion detailing the degree of success of the project and
  describing future work, that is, the next steps to take if the
  project were to continue.

### Project Presentations

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
* For each project:
    * Achieving all agreed-upon project goals: 2 robot credits
      * **Note**: If, as the project develops, a project goal appears unrealistic to achieve, the students
        and professor may renegotiate the project goals **at least one day prior to the submission deadline**.
    * Achieving some but not all of the agreed-upon project goals: 1 robot credit
    * Submitting a satsifactory project report: 1 robot credit
    * Delivering a satisfactory project presentation: 1 robot credit
* Asking questions during presentations:
  * Ask at least one question on each of the four presentation days: 2 robot credits
  * Ask at least one question on each of two presentation days: 1 robot credit
  
## <a name="essays">Essays</a>
A total of three in-class essays and one take-home essay will be assigned over the course of the semester. 
Each essay topic is posted on the course web page. In preparing for each in-class essay, each student may
make use of whatever resources they would like - readings, assignments, classmates, anything on
the Internet, or any other resource. 

Each in-class essay itself is closed-book, closed-note, and closed-device. Paper will be provided
for writing the essay, which must be submitted at the end of the class period. 

The essays will be commented upon by the instructor and returned. Each student should then
revise their essay, taking into account the instructor comments. The revised essay should be 
typed and submitted electronically. The original handwritten essay should also be resubmitted
physically. The revised essay will be due one week after the original essays are returned. 

Students are welcome to make use of additional resources when revising their essays; proper
citation should be included for each resource. Plagiarism, including submitting an essay 
rewritten by a generative AI, is strictly prohibited. Each revised essay will then be 
assessed as **Level 1** or **Level 2**, depending on the quality of the essay. Quality will 
be assessed according to the following criteria:
* Writing quality, including proper spelling, usage, and grammar.
* Demonstrated depth of understanding the essay topic.
* Appropriate use of examples from course projects.
 
### Essay Credits 
* One essay credit will be awarded per level.
* One additional essay credit will be awarded for on-time submissions of the final essays.

## <a name="finalproject">Final Project</a>
In the last three weeks of the semester, each student will undertake a final project. 
In this final project, you will program a robot that fulfills a contextualized 
purpose. A demonstration will be made of the robot's capabilities, and a paper 
reflecting upon lessons learned will be submitted as well. In keeping with the Odyssey 
Special Project guidelines, the project will require at least 30 hours of work. As with 
the other course projects, final projects will be undertaken in teams of two or three.

<hr>

## <a name="grading">Specifications Grading</a>
Final course grades are earned based on accumulated credits, as follows:

* To earn an A in the course, a student will:
  * Earn at least 30 out of 32 possible robot credits
  * Earn at least 11 out of 12 possible essay credits
  * Complete and present the final project
* To earn a B in the course, a student will: 
  * Earn at least 26 robot credits
  * Earn at least 9 essay credits
  * Complete and present the final project
* To earn a C in the course, a student will:
  * Earn at least 22 robot credits
  * Earn at least 7 essay credits
  * Complete and present the final project
* To earn a D in the course, a student will:
  * Earn at least 18 robot credits
  * Earn at least 4 essay credits

## <a name="equipment">Equipment</a>
During the second week of the semester, each student team will be assigned
an iRobot Create3 robot and a Raspberry Pi controller for that robot. As 
teams shift over the course of the semester, robots will be reassigned to the 
new teams.

The robots should be returned to the instructor at the end of the semester.
Students will be billed for any unreturned equipment.

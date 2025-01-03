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
* Write programs that control motors and process sensor information.
* Quantitatively assess the performance of a mobile robot for a given task.
* Describe both orally and in writing:
  * The design and performance of a mobile robot.
  * Assessments of deployed and fictional robots.
* Apply supervised machine learning algorithms for image classification.

## <a name="resources">Resources</a>

{% include resources.html content=site.resources %}

<hr>

# <a name="calendar">Calendar</a>

|Date|Day|Topic/Activity|Reading|Assigned|Due|
| --- | --- | --- | --- | --- | --- |
| 1/17 | Tue | Overview<br>Robots in Science Fiction |  | [Robots: Fiction and Reality]({{site.baseurl}}/projects/robots_sci_fi.html) |  |
| 1/19 | Thu | Discussion of Short Stories | [Homework](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/philip-apps/homework)<br>[Company Property](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/john-albertson/company-property)<br>[Domotica Berserker](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/paul-g-di-filippo/domotica-berserker)<br>[30 Pounds of Human Tissue](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/jennifer-campbell-hicks/30-pounds-of-human-tissue)<br>[How I Saved the Galaxy on a Limited Budget](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/aidan-doyle/how-i-saved-the-galaxy-on-a-limited-budget)<br>[Memo From the Lab of the Moral Weapon](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/h-baumgardt/memo-from-the-lab-of-the-moral-weapon)<br>[Under My Thumb](https://web.archive.org/web/20240423181154/https://dailysciencefiction.com/science-fiction/robots-and-computers/laura-ansara/under-my-thumb) |  |  |
|      |
| 1/24 | Tue | [Introduction to EV3 Kits]({{site.baseurl}}/projects/avoid.html) | [MicroPython](https://pybricks.com/ev3-micropython/) <br> [A Robust Layered Control System for a Mobile Robot]({{site.baseurl}}/readings/Robust_Control.pdf) | | Robots: Fiction and Reality |
| 1/26 | Thu |  |  |  |  |
|      |
| 1/31 | Tue | Ice-Pocalypse: No class |
| 2/2  | Thu | [Mode Selection]({{site.baseurl}}/projects/modes.html) |  |  | Introduction to EV3 Kits |
|      |
| 2/7  | Tue |  |  |  |  |
| 2/9  | Thu |  |  |  |  |
|      |
| 2/14 | Tue | [Reinforcement Learning of Behavior]({{site.baseurl}}/projects/qlearning.html) |  |   | Mode Selection |
| 2/16 | Thu |  |  |  |  |
|      |
| 2/21 | Tue | |  |  | Reinforcement Learning of Behavior |
| 2/23 | Thu | [Fuzzy Logic Behavior]({{site.baseurl}}/projects/fuzzy.html) |  |  |  |
|      |
| 2/28 | Tue | |  |  | |
| 3/2  | Thu | |  |  |  |
|      |
| 3/7  | Tue | [Action Selection with Images]({{site.baseurl}}/projects/vision1.html) |  |  | Fuzzy Logic Behavior  |
| 3/9  | Thu |  |  |  |   |
|      |
| 3/14 | Tue |  |  |  |  |
| 3/16 | Thu |  |  |  |  Action Selection with Images  |
|      |
| 3/21 | Tue | Spring Break |  |  |  |
| 3/23 | Thu | Spring Break |  |  |  |
|      |
| 3/28 | Tue | [Maps]({{site.baseurl}}/projects/vision2.html) |  |  |  |
| 3/30 | Thu |  |  |  |  |
|      |
| 4/4  | Tue |  |  |  |  |
| 4/6  | Thu |  |  |  | Maps |
|      |
| 4/11 | Tue | [Final Project]({{site.baseurl}}/projects/Final.html) |  | [Final Project Proposal]({{site.baseurl}}/projects/Final.html) |  |
| 4/13 | Thu | Final Project work |  |  |  |
|      |
| 4/18 | Tue | Final Project proposal presentations |  |  | Final Project Proposal |
| 4/20 | Thu | Final Project work |  |  |  |
|      |
| 4/25 | Tue | Final Project Progress Reports |  |  |  |
| 4/27 | Thu | Final Project work |  |  |  |
|      |
| 5/2  | Tue 2-5 pm | Final Project Presentations | | | [Final Projects]({{site.baseurl}}/projects/Final.html) |


<hr>
# <a name="assessment">Assessment</a>

## <a name="projects">Projects</a>

Every 2-3 class periods, a project will be assigned. Students will complete each project
in teams of two. In most projects, students will program their robots to perform a task using
a newly introduced concept, potentially incorporating other concepts covered previously. Some time 
will typically be available during some class periods for work on the current project.

### Project Reports
For each project, each student (even if part of a team) should submit an individual project
report. Each report includes the following:
* A project log, which includes the following for every work session:
  * Date of the work session, including start and end times.
  * Goals for the session.
  * Brief descriptions of activities undertaken.
  * Observations of activities.
  * Assessment of the degree to which session goals were met.
* Answers to project-specific questions.
* A conclusion detailing the degree of success of the project.

### Project Presentations

On the due date of each project, each team will play a video in class. The video should meet the following constraints:
* It must be between 60 and 120 seconds in duration.
* It should include brief narration of the strategy for the project.
  * Narration may be pre-recorded or provided live as the video runs.
* It should demonstrate the student's robots performing the required tasks for the week's project. Narration should contextualize each demonstrated activity.

**Note:** Presentations will only be given when the project actively involves robot programming.
There will be no presentations following the first project, as it does not involve robot programming.

## <a name="finalproject">Final Project</a>
In the last three weeks of the semester, each student will undertake a final project. 
In this final project, you will build and program a robot that fulfills a contextualized 
purpose. A public demonstration will be made of the robot's capabilities, and a paper 
reflecting upon lessons learned will be submitted as well. In keeping with the Odyssey 
Special Project guidelines, the project will require at least 30 hours of work. As with 
the other course projects, final projects will be undertaken in teams of two.

## <a name="participation">Class Participation</a>

### Presentation Questions
* Students should be prepared to answer questions after their video presentation concludes. 
* Each student is expected to ask one question on each class day that includes presentations.

## <a name="grading">Specifications Grading</a>
Each assignment is graded on a pass-fail basis. To earn a passing grade, the assignment
must be substantively complete; minor imperfections are perfectly acceptable. Final course
grades are earned based on completed passing assignments, as follows:

* To earn an A in the course, a student will:
  * Complete all seven projects
  * Complete the final project
  * Ask seven presentation questions
* To earn a B in the course, a student will: 
  * One of the following:
    * Complete any six projects
    * Complete at least five projects and partially complete two more projects.
  * Complete the final project
  * Ask five presentation questions
* To earn a C in the course, a student will:
  * One of the following:
    * Complete any four projects
    * Complete at least two projects and partially complete at least three more.
  * Complete the final project
  * Ask three presentation questions
* To earn a D in the course, a student will:
  * One of the following:
    * Complete any four projects
    * Complete at least two projects and partially complete at least three more.

## <a name="tokens">Tokens</a>
* Each student starts the semester with three **tokens**.
* Send Dr. Ferrer a message on Teams to spend a token.
* A student may spend one token in order to:
  * Submit a project after the posted deadline.
      * When you send the message to spend the token, specify a new
      deadline for that project that you plan to meet.
  * Submit a revised version of a project in the event the submission is not
    of sufficient quality to receive a grade of Complete.
* Scheduling and attending an [office hours meeting](https://drferrer.youcanbook.me)
  with Dr. Ferrer earns one additional token.
* **Note**: All late submissions/revisions must be received before 5 pm on Tuesday, 
  May 9, the last day of the semester.

## <a name="equipment">Equipment</a>
During the second week of the semester, each student team will be assigned
a Lego Mindstorms EV3 robot kit. Later in the semester, each team will also be
assigned a Kindle Fire tablet to augment the robot with computer vision 
capabilities. 

The equipment should be returned to the instructor at the end of the semester.
Students will be billed for any unreturned equipment.
<!--
Each student will need to supply a smartphone or tablet running the Android operating system.
Students for whom this presents a difficulty should contact the instructor, who will investigate
possible arrangements.

## App Download Link

[Download App](https://github.com/gjf2a/Tracker2/releases)
-->

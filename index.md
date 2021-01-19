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

**Note:** Topics are tentative and subject to change.

|Date|Day|Topic/Activity|Reading|Assigned|Due|
| --- | --- | --- | --- | --- | --- |
| 1/19 | Tue | Overview<br>Robots in Science Fiction |  | [Robots: Fiction and Reality]({{site.baseurl}}/projects/robots_sci_fi.html) |  |
| 1/21 | Thu | Discussion of Short Stories<br>Overview of Symbolic AI | [Domotica Berserker](https://dailysciencefiction.com/science-fiction/robots-and-computers/paul-g-di-filippo/domotica-berserker)<br>[30 Pounds of Human Tissue](https://dailysciencefiction.com/science-fiction/robots-and-computers/jennifer-campbell-hicks/30-pounds-of-human-tissue)<br>[Audit's Abacus](https://dailysciencefiction.com/science-fiction/robots-and-computers/robert-bagnall/audits-abacus)<br>[Care Robot](https://dailysciencefiction.com/science-fiction/robots-and-computers/eric-s-fomley/care-robot)<br>[Call Center Blues](https://dailysciencefiction.com/science-fiction/robots-and-computers/carrie-cuinn/call-center-blues)<br>[The Laughing Paradox](https://dailysciencefiction.com/science-fiction/robots-and-computers/dylan-otto-krider/the-laughing-paradox) |  |  |
| 1/26 | Tue | Symbolic vs Physical AI<br>HTN Planning | [Elephants Don't Play Chess]({{site.baseurl}}/readings/Brooks1990.pdf) | [Planning]({{site.baseurl}}/projects/planning.html) | Robots: Fiction and Reality |
| 1/28 | Thu | HTN Planning |  |  |  |
| 2/2 | Tue | Building and Programming the Robot |  | Build Robot | Planning |
| 2/4 | Thu | Driving in Patterns |  | Driving in Patterns | Build Robot |
| 2/9 | Tue | Sonar |  | Reactive Behaviors 1 | Driving in Patterns |
| 2/11 | Thu | Sonar |  |  |  |
| 2/16 | Tue | Computer Vision |  | Reactive Behaviors 2 | Reactive Behaviors 1 |
| 2/18 | Thu | Computer Vision |  |  |  |
| 2/23 | Tue | Break: no class |  |  |  |
| 2/25 | Thu | Computer Vision |  |  |  |
| 3/2 | Tue | Tracking  |  | Tracking | Reactive Behaviors 2 |
| 3/4 | Thu | Tracking  |  |  |  |
| 3/9 | Tue | Reinforcement Learning |  | Reinforcement Learning | Tracking |
| 3/11 | Thu | Reinforcement Learning |  |  |  |
| 3/16 | Tue | Combining Reactive Behaviors |  | Combining Behaviors | Reinforcement Learning |
| 3/18 | Thu | Combining Reactive Behaviors |  |  |  |
| 3/23 | Tue | Neural Networks |  | Neural Networks | Combining Reactive Behaviors |
| 3/25 | Thu | Neural Networks continued |  |  |  |
| 3/30 | Tue | Landmarks |  | Landmarks | Neural Networks |
| 4/1 | Thu | Break: no class |  |  |  |
| 4/6 | Tue | Landmarks |  |  |  |
| 4/8 | Thu | Landmarks |  |  |  |
| 4/13 | Tue | Final Project |  | Final Project Proposal | Landmarks |
| 4/15 | Thu | Final Project work |  |  |  |
| 4/20 | Tue | Final Project Progress Reports |  |  |  |
| 4/22 | Thu | Final Project work |  |  |  |
| 4/27 | Tue | Final Project Progress Reports |  |  |  |
| 4/29 | Thu | Final Project work |  |  |  |
| 5/5 | Wed | Final Project Presentations | | | Final Projects |


<hr>
# <a name="assessment">Assessment</a>

## <a name="projects">Projects</a>

Every Tuesday, a project will be assigned. Students may complete projects individually or 
in teams of two. In most projects, students will program their robots to perform a task using
a new concept introduced that week, potentially incorporating other concepts covered in 
previous weeks. Each project will typically be due the following Tuesday, with a brief
video presentation given in class. Some time will typically be available every Thursday
during the class period for work on that week's project.

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
* It must be between 80 and 90 seconds in duration.
  * For team projects, the video should be 160 to 180 seconds in duration.
* It should include brief narration of the strategy for the project.
* It should demonstrate the student's robots performing the required tasks for the week's project. Narration should contextualize each demonstrated activity.

**Note:** Presentations will only be given when the project actively involves robot programming.
There will be no presentations following the first two projects, as they do not involve robot programming.

## <a name="finalproject">Final Project</a>
In the last three weeks of the semester, each student will undertake a final project. 
In this final project, you will build and program a robot that fulfills a contextualized 
purpose. A public demonstration will be made of the robot's capabilities, and a paper 
reflecting upon lessons learned will be submitted as well. In keeping with the Odyssey 
Special Project guidelines, the project will require at least 30 hours of work. As with 
the other course projects, final projects may be undertaken either individually or in 
teams of two.

## <a name="participation">Class Participation</a>

### Presentation Questions
* Students should be prepared to answer questions after their video presentation concludes. 
* Each student is expected to ask one question on each class day that includes presentations.

### Office Hours
* Each student should schedule and attend at least three online Office Hour meetings with the instructor at some point during the semester.


## <a name="grading">Specifications Grading</a>
Each assignment is graded on a pass-fail basis. To earn a passing grade, the assignment
must be substantively complete; minor imperfections are perfectly acceptable. Final course
grades are earned based on completed passing assignments, as follows:

* To earn an A in the course, a student will:
  * Complete all ten projects
  * Complete the final project
  * Submit a course feedback form
  * Ask six presentation questions
  * Schedule and attend at least three Office Hours meetings
* To earn a B in the course, a student will: 
  * Complete any seven projects
  * Complete the final project
  * Submit a course feedback form
  * Ask four presentation questions
  * Schedule and attend at least two Office Hours meetings
* To earn a C in the course, a student will:
  * Complete any five projects
  * Complete the final project
  * Submit a course feedback form
  * Ask two presentation questions
  * Schedule and attend at least one Office Hours meeting
* To earn a D in the course, a student will:
  * Complete any four projects

### Revising submitted work
If a submitted project is not of sufficient quality to receive a passing grade:
* The instructor will give feedback identifying revisions that, if applied, would result in a passing grade.
* The student will schedule and attend an Office Hours meeting to discuss the necessary revisions and establish a deadline for their submission.
* If the student submits the revisions by the agreed deadline, the revised project will receive a passing grade.

## <a name="latedays">Late Policy</a>
If a student needs an extension, the instructor must be notified by email by 4 pm on the 
day prior to the due date. This notification email must state the duration of the requested 
extension. The instructor reserves the right to decline a request for an extension, but 
the intention is that most requests for extensions will be granted.

## <a name="equipment">Equipment</a>
Each student will be issued the following equipment:
* One robot kit, containing:
  * Two motors and wheels
  * A battery case
  * A plexiglas chassis
* One breadboard
* One motor control chip
* One Arduino board

This equipment should be returned to the instructor at the end of the semester. Students will
be billed for any unreturned equipment.

Each student will need to supply a smartphone or tablet running the Android operating system.
Students for whom this presents a difficulty should contact the instructor, who will investigate
possible arrangements.
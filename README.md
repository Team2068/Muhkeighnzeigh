# 2023 Charged Up

This is [The Metal Jackets](https://www.metaljackets.org/) robot code for the 2023 FIRST Robotics Competition, Charged Up.

## Docs / Resources

* [FIRST Robotics Competition Docs](https://docs.wpilib.org/en/latest/)
* [SPARK MAX](https://www.revrobotics.com/sparkmax-software/) and [Status Lights/Codes](https://www.revrobotics.com/sparkmax-quickstart/#status-led)
* [REV Robotics Analog Pressure Sensor](https://www.revrobotics.com/rev-11-1107/)
* [LimeLight](https://docs.limelightvision.io/en/latest/)
* [REV Robotics Color Sensor](https://www.revrobotics.com/rev-31-1557/)

## Coding Style

Until we define our own coding style, we'll use [Google's](https://google.github.io/styleguide/javaguide.html) and record any deviations here.

## VSCode and the WPILib Dev Environment

Follow the [WPILib Installation Guide](https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html) to set up your environment.

### Extensions

You'll need the following extensions:

#### Mandatory

* TBD

#### Optional

* TBD

### Setting up the JDK

You should've already set the location of your JDK installation in the above tutorial. If you need to set it again, here are the instructions.
1. Navigate to `File -> Preferences -> Settings`
1. Search for "jdk" in the search bar
1. Click `Java Configuration` on the left-hand sidebar. The only setting visible should be `Java: Home`
1. Click on `Edit in settings.json`
1. The right-hand side stores any settings made by the user. Add a line like this at the end of the file: `"java.home": "/Path/To/JDK/Installation"`
    - If you don't know where your JDK installation is, it's probably in `C:\Users\<Your Username>\frc2020\jdk`.
1. You're done! Wait a bit for the Java Language Server to start up and recognize your project (you should see a little spinning icon at the bottom left of your screen), then test it out by clicking on a variable type (like `Module` or `Drive` or `Double`) and pressing <kbd>F12</kbd>. If all goes well, you should be taken to the definition of that class.

### Opening Projects

It's pretty easy. `File -> Open Folder...`, then navigate to the repository you have cloned (The folder named `2020CommandCode` this year). 

### Want to learn more?

[Code Navigation](https://code.visualstudio.com/docs/editor/editingevolved)

[Basic Editing](https://code.visualstudio.com/docs/editor/codebasics)

## Building and Deploying

- Run these commands from Git Bash (or through the VS Code interface)
- To build, run `./gradlew build`
- To deploy to the robot, run `./gradlew deploy`
    - Remember to **build** before you **deploy**
- To do both at once, run `./gradlew build deploy`


## Contributing

Here's how to get your code into the main robot repository:

### If you've just joined the team:

1. Make an account on [GitHub](https://github.com/).
2. Ask one of the robot programming leads to add your account to the Team2068 robot programming team.

### If it's the first time you've contributed to this repo:

1. Clone the repo to your computer - `git clone https://github.com/Team2068/2020CommandCode.git`

### Any time you want to make a change:

We use a feature branch workflow. You can read more about that [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

1. Create and checkout a new branch.
  * `git checkout -b <your_branch_name>`, where <your_branch_name> is a descriptive name for your branch. For example `fix-shooter-wheel`, `two-ball-auto`, or `climbing`. Use dashes in the branch name, not underscores.
2. Make whatever code changes you want/need/ to make. Be sure to write tests for your changes!
3. Commit your work locally.
  * If you're on a shared laptop set the author of the commit message by: `git commit --author="Author Name <email@address.com>"`
  * Try to make your commits as atomic (small) as possible. For example, moving functions around should be different from adding features, and changes to one subsystem should be in a different commit than changes to another subsystem.
  * Follow [these](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) conventions for commit messages. Or else.
  * If your change is anything more than a few lines or small fixes, don't skip the extended description. If you are always using `git commit` with the `-m` option, stop doing that.
4. Push to your branch.
  * `git push origin <your_branch_name>`.
5. Submit a pull request.
  1. Log into Github.
  1. Go to our repo.
  1. Select the branch that you just pushed from the "Branch" dropdown menu.
  1. Click "New Pull Request".
  1. Review the changes that you made.
  1. Explain what and why you did the things you're trying to commit.  Make the reviewer's life easier.
  1. If you are happy with your changes, click "Create Pull Request".
6. Wait
  * People must review (and approve of) your changes before they are merged - master is locked to any pull requests that don't have at least 2 reviews.
    * Specifically, the rules are that one of the following two conditions must be true for it to get merged:
      1. 1 mentor and 1 other person have approved
      1. 2 experienced students and one other person have approved
  * If there are any concerns about your pull request, fix them. Depending on how severe the concerns are, the pull request may be merged without it, but everyone will be happier if you fix your code. 
To update your PR, just push to the branch you made before.
  * Don't dismiss someone's review when you make changes - instead, ask them to re-review it.
7. Merge your changes into master
  * If there are no conflicts, push the "merge" button.
  * If there are conflicts, fix them locally on your branch, push them, wait for Travis CI to pass, and then merge.
8. ???
9. Profit

### Code Review

Code reviews are one of the hardest things to get right.  There's a lot of discussion about this online, just [ddg](https://duckduckgo.com/?q=code+review+best+practices&t=brave&ia=web) it.  

You're putting yourself out there and asking your peers if your code is ready to be merged into master.  It feels like you're asking your peers to tell you if you're good enough or smart enough.  Trust that everyone feels this way, even your mentors with decades of programming experience.  Remember this when you're reviewing someone elses **CODE**.

There will be zero tolerance for attacking anyone in a code review.

#### Why

I think this list from [ProgrammerFriend](https://programmerfriend.com/code-review-best-practices/) sums it up nicely:

* Improve code quality
* Consistency in your projects
* Finding bugs when they're fresh
* Learning (by getting code reviewed) and Teaching (by reviewing other’s code)
* Creating a sense of mutual responsibility
* Being aware of changes to the code
* Keeping everyone honest to maintain the highest quality of code
* In general finding way better solutions to problems

#### What

* Correctness - Does the code do what the commit message suggest it does?
* Readability - Prefer **clarity** over **cleverness**, on variable and method names that are unambiguous and convey intent.
* Maintainability - Is code written, or commented, in such a way that someone else (or yourself a week or two later) understands why you did things the way you did?

#### How

Again, this list from [ProgrammerFriend](https://programmerfriend.com/code-review-best-practices/) sums it up nicely:

* Be friendly
* Review the code not the coder
* Give short and precise comments about your findings.  Make suggestions, link to documentation, generally be helpful.

Prefer:

"WDYT about `reverse` instead of `blah` here?  It might make it easier to remember"

Over:

"`blah` sucks, let's make this `reverse`"

## Credits

* [ILITE robotics](https://github.com/iliterobotics/FRC-Robot-2019) and by extension [FRC Team 1678: Citrus Circuits](https://github.com/frc1678) for their efforts in writing outstanding contributing guidelines
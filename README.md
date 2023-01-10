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

- Clone the repo to your computer using the current URL.

### Any time you want to make a change:

We use a feature branch workflow. You can read more about that [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow).

1. Create and checkout a new branch.
  - `git checkout -b <your_branch_name>`, where <your_branch_name> is a descriptive name for your branch. For example `fix-shooter-wheel`. Use dashes in the branch name, not underscores.
2. Make whatever code changes you want/need/ to make. Be sure to write tests for your changes!
3. Commit your work locally.
  - If you're on a shared laptop set the author of the commit message by: `git commit --author="Author Name <email@address.com>"`
  - Try to make your commits small, like example. For example, moving functions around should be different from adding features, and changes to one subsystem should be in a different commit than changes to another subsystem.
  - Just give a short summary. or follow [these](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) for conventions.
  - If your change is anything more than a few lines or small fixes, don't skip the extended description. If you are always using `git commit` with the `-m` option, stop doing that.
4. Push to your branch.
  - `git push origin <your_branch_name>`.
5. Submit a pull request.
  1. Select the branch that you just pushed from the "Branch" dropdown menu.
  2. Click "New Pull Request".
  3. Review the changes that you made.
  4. Explain what and why you did the things you're trying to commit.  Make the reviewer's life easier.
  5. If you are happy with your changes, click "Create Pull Request".
6. Wait
 - People must review (and approve of) your changes before they are merged - master is locked to any pull requests that don't have at least 2 reviews.
 - If there are any concerns about your pull request, fix them; just do it bro;
To update your PR, just push to the branch you made before.
  * Don't dismiss someone's review when you make changes - instead, ask them to re-review it.
7. Merge your changes into master
  * If there are no conflicts, push the "merge" button.
  * If there are conflicts, fix them locally on your branch, push them, wait for Travis CI to pass, and then merge.

### Code Review

Code reviews are one of the hardest things to get right.  There's a lot of discussion about this online, and those before us said [ddg](https://duckduckgo.com/?q=code+review+best+practices&t=brave&ia=web) i-
- Why:  So code breakyless.
- How: Give short and precise comments about your findings.  Make suggestions, link to documentation, generally be helpful.
- What are you looking for:
    1. Does it pass CI?
    2. Does it have Tests?
    3. Does it perform well?
    4. Is it readible?
    5. ~~You should probably check this before making a PR~~ Has it actually been tested on the robot?

Welcome!
This GitHub repository contains the source code that is used to build an Android app to control a FIRST Tech Challenge competition robot. To use this SDK, download/clone the entire project to your local computer.

If you are new to the FIRST Tech Challenge software and control system, you should visit the online wiki to learn how to install, configure, and use the software and control system:

      https://github.com/ftctechnh/ftc_app/wiki

Note that the wiki is an "evergreen" document that is constantly being updated and edited. It contains the most current information about the FIRST Tech Challenge software and control system.

Downloading the Project
It is important to note that this repository is large and can take a long time and use a lot of space to download. If you would like to save time and space, there are some options that you can choose to download only the most current version of the Android project folder:

If you are a git user, FIRST recommends that you use the --depth command line argument to only clone the most current version of the repository:
            git clone --depth=1 https://github.com/ftctechnh/ftc_app.git

Or, if you prefer, you can use the "Download Zip" button available through the main repository page. Downloading the project as a .ZIP file will keep the size of the download manageable.

You can also download the project folder (as a .zip or .tar.gz archive file) from the Downloads subsection of the Releases page for this repository.

Once you have downloaded and uncompressed (if needed) your folder, you can use Android Studio to import the folder ("Import project (Eclipse ADT, Gradle, etc.)").

Getting Help
User Documentation and Tutorials
FIRST maintains an online wiki with information and tutorials on how to use the FIRST Tech Challenge software and robot control system. You can access the wiki at the following address:

      https://github.com/ftctechnh/ftc_app/wiki

Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online. Visit the following URL to view the FTC SDK documentation as a live website:

      http://ftctechnh.github.io/ftc_app/doc/javadoc/index.html

Documentation for the FTC SDK is also included with this repository. There is a subfolder called "doc" which contains several subfolders:

The folder "apk" contains the .apk files for the FTC Driver Station and FTC Robot Controller apps.
The folder "javadoc" contains the JavaDoc user documentation for the FTC SDK.
Online User Forum
For technical questions regarding the SDK, please visit the FTC Technology forum:

      http://ftcforum.usfirst.org/forumdisplay.php?156-FTC-Technology

Release Information
Version 3.6 (built on 17.12.18)

Changes include:

Blocks Changes
Uses updated Google Blockly software to allow users to edit their op modes on Apple iOS devices (including iPad and iPhone).
Improvement in Blocks tool to handle corrupt op mode files.
Autonomous op modes should no longer get switched back to tele-op after re-opening them to be edited.
The system can now detect type mismatches during runtime and alert the user with a message on the Driver Station.
Updated javadoc documentation for setPower() method to reflect correct range of values (-1 to +1).
Modified VuforiaLocalizerImpl to allow for user rendering of frames
Added a user-overrideable onRenderFrame() method which gets called by the class's renderFrame() method.
Release Information
Version 3.5 (built on 17.10.30)
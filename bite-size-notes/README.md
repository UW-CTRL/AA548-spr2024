# bite-size-notes

You can write your "bite-size notes" using Markdown and all the nice formatting functionalities that Markdown provides.

## What to include in your notes

These notes are supposed to provide the reader with a quick deep dive into a particular topic or concept. To provide some guidelines on what topic/concept to choose, you can choose any topic that is related to that week's lecture. This can include something that was covered extensively during lectures or a passing reference that was made but was not deeply covered. Naturally, your choice of topic will depend on your level of familiarity. Diversity on topic choices is strongly encouraged. An anti-example is summarizing an entire lecture's worth of material that spans multiple topics/concepts.

In constructing your notes, you should include the following aspects:
- **Scope + objectives**: What should one expect to learn from reading these notes? Think about what is the purpose of these notes, what questions will be answered, and what will the reader gain from reading these notes.
- **Introduction**: Introduce the topic. Include things like how it is relevant to controls, in what situation would a reader find this useful, is there some historical context that's important to know, is this related to other topics?
- **Preliminaries**: Set up the mathematics needed to describe the topic/concept. Introduce definitions, notation, theorems, etc.
- **Main body**: Describe the topic/concept at a suitable depth given your selected scope and objectives. Consider including:
   - Figures and diagrams
   - Code snippets
   - Video/gif
   - Derivations
   - Concrete examples (e.g., with a specific system and with actual numbers)
   - Anything else that you think will be helpful!
- **Conclusion**: Summarize the key takeaways, and also mention what was not covered in these notes, and perhaps what are references the reader read to learn more.
- **References**: Please include references to any material you cite, or used in creating these notes.

## Role as a reviewer
  
As a reviewer, you should provide feedback on what you think the writer did well in and ways the notes can be improved. Things to think about:
- Did you learn something new or gain new insight into something you thought you already knew?
- Were there any parts where it wasn't clear? Or confusing? How could it be addressed?
- Was a non-trivial statement made without justification?
- Is the figure self-contained? Are labels provided?
- Is it content technically correct? Are the equations correct? Typos?

Please keep in mind that we are all here to learn, and develop our knowledge and skills. So please be polite and civil in your feedback.

## Revising notes

Read the reviewer's comments and see if you can address them in your revisions. It may not make sense to address everything but try your best given the scope and purpose of your notes. If the reviewer misinterpreted some aspects and therefore their feedback is not completely valid, try to understand how the misinterpretation could have happened and see if you can address that. 


## Naming convention guidelines
To help keep all the filenames organized, please use the following naming convention. 
On the sign-up spreadsheet, if you signed up for `week X` and `row y` then please use the following filename `week-X-y-short-topic-name.md`

For example, `week-1-a-linearizing-dynamics.md` or `week-4-c-timevarying-lqr.md`

Note: We avoid using your full name as this repo will be public.

## Submitting your initial notes

0. Make a github account if you do not already have one. 

1.  Fork this repo onto your github account by clicking the "Fork" button at the upper right of the main repo page; this will create a copy of the repo under your own GitHub account.

2. In your forked repo (`your-own-username/AA548-spr2024`), go to `bite-size-notes` folder and click `Add file` make a new file with the appropriate filename `week-X-y-short-topic-name.md`

3. You can edit directly in the browser (easiest if you are not very familiar with git). Otherwise, you can clone the forked repo locally on your machine and edit the markdown file in your favorite text editor (e.g., VSCode). You can continue to edit and commit changes (basically saving a version), and add figures to the `figs` folder.

4. When you are finished with your file, make a pull request to submit your work. On the top, click on the `Pull requests` tab, and click the green `New pull request` button on the top right. You will see all the commits and new files you have made. Check that you are comparing the head repository `your-own-username/AA548-spr2024` with the base repository `UW-CTRL/AA548-spr2024`.
   
5. Click `Create new pull request`. Use `week-X-y` as the pull request title, and you can add whatever details you like in the description that would be helpful for reviewers reading your file. Then click `Create pull request`. (Do not merge your pull request yet!).
  
6. You are finished submitting your initial version of your bite-size-notes! You can go to `UW-CTRL/AA548-spr2024` repo to look at the pull request you just made. The grader will check if you have submitted your pull request on time.


## Reviewing other notes.

1. In the `UW-CTRL/AA548-spr2024` repo, click on the `Pull requests` tab. You should see all the pull requests made by the other students in that week.

2. Click on the one you are assigned to review. You should see a conversation of all the commits the author of the file made.
   
3. Click on `Files changed` and you can see all the changes that the author made.
4. There are little `+` signs at the start of each line. Click on that `+` sign if you want to leave a comment on that line. And click `Start a review`. Continue making comments elsewhere on the file.
5. When you are done with comments, on the top right, click `Review changes`. Leave a general comment to the author, including
    - What you liked about their notes
    - What are areas of improvements/suggestions
Then click `Request changes` and submit the review.
7. That's it! You have provided feedback for the author. The grader will check if you have provided your feedback on time.

## Incorporating feedback from reviewers.

0. Go to the pull requests tab on `UW-CTRL/AA548-spr2024` and click on your own pull request to see the comments the reviewer made.
1. Back to your own forked repo `your-own-username/AA548-spr2024` go back to your file, and make the necessary edits. Commit your changes.
2. Going back to `UW-CTRL/AA548-spr2024` and looking into your pull request, you can see that your changes are automatically synced in `Files changed`.
3. Then you can go back to the comments left by the reviewer acknowledging you have addressed their comments.
4. That's it! The grader will review and check you have submitted your revised notes with the reviewer's comments addressed and then perform the merge.

## Other notes

You can always get the latest notes from `UW-CTRL/AA548-spr2024` on your own repo by clicking the `Sync fork` button. Since everyone should be making changes on their own on separate files, there (in theory) should not be any conflicts.


## Tips and tricks for markdown

You can include a figure like this:
![alt text](figs/leung_cat.jpg "Title")

You can make subsubheadings like this
### subsubheading

or even subsubsubheading
#### subsubsubheading

You can add code snippets like this:
```
import numpy as np

def foo(x):
    print(x)
```

Or you can write equations using LaTeX notation like this:

You can write equations inline like this $x=\frac{2}{7}$ or on it's own line

$$ y=\sin(x) + \sum_{i=0}^T x^TQx$$

But don't forget to add references and acknowledge the sources you used to create these notes

[1] Optimal Control and Estimation by Robert F. Stengel.

[2] [Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation](https://underactuated.csail.mit.edu/) by Russ Tedrake 

[3] Optimal Control Theory: An Introduction by Donald E. Kirk.

[4] [Linear Systems Theory](https://web.ece.ucsb.edu/~hespanha/linearsystems/) by Joao Hespanha

[4] [Safe Autonomy with Control Barrier Functions](https://link.springer.com/book/10.1007/978-3-031-27576-0) by Wei Xiao, Christos G. Cassandras, and Calin Belta.

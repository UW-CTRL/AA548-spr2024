# bite-size-notes

You can write your "bite-size notes" using Markdown and all the nice formatting functionalities that Markdown provides.

## Naming convention guidelines
To help keep all the filenames organized, please use the following naming convention. 
On the sign-up spreadsheet, if you signed up for `week X` and `row y` then please use the following filename `week-X-y-short-topic-name.md`

For example, `week-1-a-linearizing-dynamics.md` or `week-4-c-timevarying-lqr.md`

Note: We avoid using your full name as this repo will be public.

## Submitting your work

0. Make a github account if you do not already have one. 

1.  fork this repo onto your github account.

2. In your forked repo, make a new file in the `bite-size-notes` folder with the appropriate filename.

3. You can edit directly in the browser (easiest if you are not very familiar with git). Otherwise, you can clone the forked repo locally on your machine and edit the markdown file in your favorite text editor (e.g., VSCode).

4. Make a pull request to submit your work. Use `week-X-y` as the pull request title, and you can add whatever details you like in the description.

[TODO: make this more detailed / iron out the details]

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

.. contributing

How to contribute
=================

OpenDroneMap relies on community contributions. You can contribute in many ways, even if you are not a programmer.

Community Forum
---------------

If you are looking to get involved, are stuck on a problem, or want to reach out, `the forum <https://community.opendronemap.org/>`_ is a great place to start. You may find your questions already answered or else you can find other useful tips and resources. You can also contribute your open access datasets for others to explore. It is a good place go before submitting bug reports or getting in touch with developers before writing a new feature. In addition to the forum, you can reach us on `gitter <https://gitter.im/OpenDroneMap/OpenDroneMap/>`_.

Reporting Bugs
--------------

Bugs are tracked as Github issues. Please create an issue in the repository and tag it with the Bug tag.

Explain the problem and include additional details to help maintainers reproduce the problem:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. For example, start by explaining how you run ODM (Docker, Vagrant, etc), e.g. which command exactly you used in the terminal. When listing steps, **don't just say what you did, but explain how you did it.**
* **Provide specific examples to demonstrate the steps.** Include links to files or GitHub projects, or copy/pasteable snippets, which you use in those examples. If you're providing snippets in the issue, use `Markdown code blocks <https://help.github.com/articles/markdown-basics/#multiple-lines>`_.
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why.**
* **Include screenshots and animated GIFs** which show you following the described steps and clearly demonstrate the problem. You can use `this tool to record GIFs on macOS and Windows <http://www.cockos.com/licecap/>`_, and `this tool <https://github.com/colinkeenan/silentcast>`_ or `this one <https://github.com/GNOME/byzanz>`_ on Linux.
* **If the problem is related to performance,** please post your machine's specs (host and guest machine).
* **If the problem wasn't triggered by a specific action,** describe what you were doing before the problem happened and share more information using the guidelines below.

Include details about your configuration and environment:

* **Which version of ODM are you using?** A stable release? a clone of master?
* **What's the name and version of the OS you're using?**
* **Are you running ODM in a virtual machine or Docker?** If so, which VM software are you using and which operating systems and versions are used for the host and the guest?

Template For Submitting Bug Reports
```````````````````````````````````
::

    [Short description of problem here]

    **Reproduction Steps:**

    1. [First Step]
    2. [Second Step]
    3. [Other Steps...]

    **Expected behavior:**

    [Describe expected behavior here]

    **Observed behavior:**

    [Describe observed behavior here]

    **Screenshots and GIFs**

    ![Screenshots and GIFs which follow reproduction steps to demonstrate the problem](url)

    **ODM version:** [Enter ODM version here]
    **OS and version:** [Enter OS name and version here]

    **Additional information:**

    * Problem started happening recently, didn't happen in an older version of ODM: [Yes/No]
    * Problem can be reliably reproduced, doesn't happen randomly: [Yes/No]
    * Problem happens with all datasets and projects, not only some datasets or projects: [Yes/No]

Pull Requests
-------------

* Include screenshots and animated GIFs in your pull request whenever possible.
* Follow the PEP8 Python Style Guide.
* End files with a newline.
* Avoid platform-dependent code:
    * Use require('fs-plus').getHomeDirectory() to get the home directory.
    * Use path.join() to concatenate filenames.
    * Use os.tmpdir() rather than /tmp when you need to reference the temporary directory.
* Using a plain return when returning explicitly at the end of a function.
    * Not return null, return undefined, null, or undefined


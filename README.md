# Engineering Portfolio — Starter Site

A blank, plain HTML/CSS/JS portfolio site. Every file sits at the top
level of the repo — no subfolders — which avoids the "GitHub web
upload silently drops folders" issue some browsers/upload flows run
into. No build tools, no frameworks, no installs required.

## Files

```
index.html              Homepage: contact info + clickable project grid
projects.html           All projects as quick-read paragraph summaries
resume.html             Résumé as a webpage (optional PDF download button)
project-template.html   Blank template — duplicate this for new projects
project-01.html         Sample project page (edit or delete)
project-02.html         Sample project page (edit or delete)
style.css               All styling — one file, heavily commented
main.js                 Mobile nav toggle + active-link highlight
.nojekyll               Tells GitHub Pages to serve the site as-is
.gitignore              Ignores OS/editor junk files
```

Photos and your résumé PDF aren't included yet — add them yourself
(see "Adding images" and "Adding your résumé" below). Everything —
HTML, CSS, JS, and eventually your images/PDF — lives in this one
flat folder.

## Site map

- **Home** (`index.html`) — lands here first. Contact info sits right
  below the nav bar, then a grid of project cards (photo, title,
  skills used). Clicking a card opens that project's full page.
- **Projects** (`projects.html`) — the same projects, but as short
  paragraphs so someone can read the whole thing without clicking
  through. Each entry also links to the same full project page.
- **Résumé** (`resume.html`) — your résumé as an actual webpage
  (summary, experience, education, skills), with an optional
  "Download as PDF" button if you keep a PDF version too.
- **Project pages** (`project-01.html`, `project-02.html`, etc.) —
  one page per project with a hero image, overview, gallery, and
  prev/next links to the other projects.

## Viewing the site locally

There's no server or build step needed. Just open `index.html` in a
web browser.

## Uploading to GitHub

Because every file is flat (no folders to navigate into), you can
either:

- **Drag and drop all the files at once** into the GitHub web
  uploader ("Add file → Upload files"), or
- Use `git` from the command line:
  ```
  git init
  git add .
  git commit -m "Initial commit"
  git branch -M main
  git remote add origin https://github.com/your-username/your-repo.git
  git push -u origin main
  ```

Either way, if a page loads without any styling on GitHub Pages, the
most common cause is `style.css` (or `main.js`) not actually having
made it into the repo — check the repo's file list on GitHub and
confirm `style.css` shows up alongside `index.html` at the top level.

## Deploying with GitHub Pages

1. Push all these files to a GitHub repository (they should sit at
   the repo root).
2. In the repo, go to **Settings → Pages**.
3. Under **Build and deployment**, set **Source** to "Deploy from a
   branch," pick your branch (usually `main`), and set the folder to
   `/ (root)`.
4. Save. GitHub will give you a URL like
   `https://your-username.github.io/your-repo-name/` — it can take a
   minute or two to go live after the first push.

A couple of things that matter specifically for GitHub:

- **File names are case-sensitive on GitHub Pages**, even if they
  weren't on your computer. If you rename or add a file, make sure
  every `href`/`src` that points to it matches the exact case.
- **`index.html` must stay named exactly that** at the repo root —
  it's what loads when someone visits the site's root URL.
- The included `.nojekyll` file tells GitHub Pages to serve the site
  exactly as-is, skipping GitHub's default Jekyll processing step.

## Adding text

Every spot meant for your own content is marked with an `EDIT ME`
comment in the HTML.

## Adding images

Photo spots are dashed boxes (`<div class="img-placeholder">`) with
a suggested filename and size right on them. Since there are no
subfolders, project photos just sit in this same top-level folder
alongside the HTML files. Suggested naming (matches what the
placeholders already reference):

```
project-01-thumb.jpg      thumbnail used on the homepage card
project-01-hero.jpg       large image at the top of the project page
project-01-gallery-1.jpg  supporting gallery images
project-01-gallery-2.jpg
project-01-gallery-3.jpg
project-01-gallery-4.jpg

project-02-thumb.jpg
project-02-hero.jpg
...and so on for each project.
```

Once a photo exists, open the relevant HTML file and replace the
matching `<div class="img-placeholder">...</div>` block with:

```html
<img src="project-01-hero.jpg" alt="Describe what's shown in the photo">
```

Recommended sizes (not required, just keeps file sizes reasonable):

| Image type  | Recommended size |
| ----------- | ----------------- |
| Thumbnails  | 800×600           |
| Hero images | 1600×900          |
| Gallery     | 1200×900          |

## Adding your résumé

Put your résumé PDF in this same folder and name it exactly
`resume.pdf`. The "Download as PDF" button on `resume.html` already
points to `resume.pdf`, so nothing else needs to change once the
file is there.

If you'd rather link to an external résumé (e.g. Google Drive or
LinkedIn) instead, open `resume.html`, find the `<a>` tag with
`href="resume.pdf"`, and replace that `href` with your external link.

## Adding a new project page

A new project needs to be added in **three** places so it shows up
everywhere consistently:

1. Copy `project-template.html` and rename it, e.g. `project-03.html`.
   Fill in every `EDIT ME` spot: title, spec strip
   (role/tools/timeline/team), overview text, and images. Update the
   "prev / next" links at the bottom to connect it to the projects
   next to it in your list.
2. Open `index.html`, duplicate one `<a class="project-card">` block
   in the "Projects" grid, and point it at your new file. Update the
   thumbnail placeholder and the skills tags.
3. Open `projects.html`, duplicate one `<article class="project-entry">`
   block, point it at the same new file, and write the paragraph
   summary.

No rebuild step needed for any of this — just save and refresh the
browser (or push to GitHub if you're viewing the live site).

## Editing styles (colors, fonts, spacing)

Everything is controlled from the top of `style.css`, inside the
`:root { ... }` block. Change a value there (a color hex code, a
font name, a spacing size) and it updates across every page, since
all pages share this one stylesheet.

## Editing the nav bar or footer

The nav bar and the footer are repeated at the top/bottom of every
page: Home, Projects, and Résumé. If you add another top-level page
later, update the `<nav>` block on every page to match.

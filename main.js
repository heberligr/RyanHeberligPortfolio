/* =============================================================
   MAIN.JS
   Small, framework-free behaviors shared by every page:
     1. Mobile nav toggle (hamburger button opens/closes menu)
     2. Active-link highlighting (bolds the nav link for the
        current page, based on a data-page attribute you set
        on the <body> tag of each page — see any HTML file's
        <body data-page="..."> for reference)
   No build step needed — just edit this file directly and
   refresh the browser.
   ============================================================= */

document.addEventListener('DOMContentLoaded', function () {

  /* ---------- 1. Mobile nav toggle ---------- */
  var toggleBtn = document.querySelector('.nav-toggle');
  var navLinks = document.querySelector('.nav-links');

  if (toggleBtn && navLinks) {
    toggleBtn.addEventListener('click', function () {
      var isOpen = navLinks.classList.toggle('is-open');
      toggleBtn.setAttribute('aria-expanded', isOpen ? 'true' : 'false');
    });
  }

  /* ---------- 2. Active-link highlighting ---------- */
  // Each page's <body> tag has a data-page attribute, e.g.
  //   <body data-page="about">
  // and each nav link has a matching data-page attribute, e.g.
  //   <a href="projects.html" data-page="projects">Projects</a>
  // This just matches them up and adds the .is-active class.
  var currentPage = document.body.getAttribute('data-page');
  if (currentPage) {
    var links = document.querySelectorAll('.nav-links a[data-page]');
    links.forEach(function (link) {
      if (link.getAttribute('data-page') === currentPage) {
        link.classList.add('is-active');
      }
    });
  }

});

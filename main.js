/* =============================================================
   MAIN.JS
   Small, framework-free behaviors shared by every page:
     1. Mobile nav toggle (hamburger button opens/closes menu)
     2. Active-link highlighting (bolds the nav link for the
        current page, based on a data-page attribute you set
        on the <body> tag of each page — see any HTML file's
        <body data-page="..."> for reference)
     3. Gallery lightbox (click a gallery photo to view it full
        size, uncropped, and step through the rest of that
        gallery — see any project page's .gallery-grid)
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

  /* ---------- 3. Gallery lightbox ---------- */
  // Every project page's .gallery-grid shows photos cropped to a
  // neat 4:3 grid (see style.css section 15). Clicking a photo
  // opens it here at full size, uncropped, with prev/next arrows,
  // arrow-key navigation, and swipe support on touch devices.
  // This is entirely self-contained — it builds the overlay once
  // and wires up whatever .gallery-grid elements exist on the
  // current page, so a new project page with its own gallery gets
  // this behavior automatically with no extra markup or script tags.
  var galleries = document.querySelectorAll('.gallery-grid');

  if (galleries.length) {
    // Build the lightbox overlay once and append it to the page.
    var overlay = document.createElement('div');
    overlay.className = 'lightbox-overlay';
    overlay.hidden = true;
    overlay.innerHTML =
      '<button class="lightbox-close" aria-label="Close">&times;</button>' +
      '<button class="lightbox-prev" aria-label="Previous image">&#8249;</button>' +
      '<img class="lightbox-img" src="" alt="">' +
      '<button class="lightbox-next" aria-label="Next image">&#8250;</button>' +
      '<span class="lightbox-caption"></span>' +
      '<span class="lightbox-counter"></span>';
    document.body.appendChild(overlay);

    var lightboxImg = overlay.querySelector('.lightbox-img');
    var captionEl = overlay.querySelector('.lightbox-caption');
    var counterEl = overlay.querySelector('.lightbox-counter');
    var currentImages = [];
    var currentIndex = 0;

    function showImage(index) {
      // wraps around in both directions, so "prev" from the first
      // photo loops to the last one and vice versa
      currentIndex = (index + currentImages.length) % currentImages.length;
      var sourceImg = currentImages[currentIndex];
      lightboxImg.src = sourceImg.src;
      lightboxImg.alt = sourceImg.alt || '';
      // If this photo sits inside a <figure> with a <figcaption>,
      // show that same caption text here too.
      var figure = sourceImg.closest('figure');
      var figcaption = figure ? figure.querySelector('figcaption') : null;
      captionEl.textContent = figcaption ? figcaption.textContent : '';
      counterEl.textContent = (currentIndex + 1) + ' / ' + currentImages.length;
    }

    function openLightbox(images, startIndex) {
      currentImages = images;
      overlay.classList.toggle('is-single', images.length <= 1);
      showImage(startIndex);
      overlay.hidden = false;
      document.body.style.overflow = 'hidden'; // pause background scroll while open
    }

    function closeLightbox() {
      overlay.hidden = true;
      document.body.style.overflow = '';
    }

    // Wire up every photo in every gallery on this page. Each
    // gallery keeps its own image list, so clicking a photo only
    // ever steps through the other photos in that same gallery.
    galleries.forEach(function (gallery) {
      var imgs = Array.prototype.slice.call(gallery.querySelectorAll('img'));
      imgs.forEach(function (img, index) {
        img.addEventListener('click', function () {
          openLightbox(imgs, index);
        });
      });
    });

    overlay.querySelector('.lightbox-close').addEventListener('click', closeLightbox);
    overlay.querySelector('.lightbox-prev').addEventListener('click', function () {
      showImage(currentIndex - 1);
    });
    overlay.querySelector('.lightbox-next').addEventListener('click', function () {
      showImage(currentIndex + 1);
    });

    // Click the dark backdrop itself (not the photo or the
    // buttons) to close, same as most lightboxes.
    overlay.addEventListener('click', function (e) {
      if (e.target === overlay) closeLightbox();
    });

    // Keyboard support while the lightbox is open.
    document.addEventListener('keydown', function (e) {
      if (overlay.hidden) return;
      if (e.key === 'Escape') closeLightbox();
      if (e.key === 'ArrowLeft') showImage(currentIndex - 1);
      if (e.key === 'ArrowRight') showImage(currentIndex + 1);
    });

    // Basic swipe support for touch devices: swipe left/right on
    // the photo to move to the next/previous image.
    var touchStartX = null;
    overlay.addEventListener('touchstart', function (e) {
      touchStartX = e.changedTouches[0].clientX;
    });
    overlay.addEventListener('touchend', function (e) {
      if (touchStartX === null) return;
      var deltaX = e.changedTouches[0].clientX - touchStartX;
      var SWIPE_THRESHOLD = 40; // pixels — ignore small accidental drags
      if (deltaX > SWIPE_THRESHOLD) showImage(currentIndex - 1);
      else if (deltaX < -SWIPE_THRESHOLD) showImage(currentIndex + 1);
      touchStartX = null;
    });
  }

});

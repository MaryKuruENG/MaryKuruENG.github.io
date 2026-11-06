(function () {
  var reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var sel = '.ms-hero,.ms-card,.ms-stat,.archive__item,.grid__item,.archive h1:not(.page__title),.page__content h2';
  var els = document.querySelectorAll(sel);

  function count(el) {
    var end = parseFloat(el.getAttribute('data-count'));
    var suf = el.getAttribute('data-suffix') || '';
    var dec = (el.getAttribute('data-count').split('.')[1] || '').length;
    var t0 = null;
    function step(t) {
      if (!t0) t0 = t;
      var p = Math.min((t - t0) / 1300, 1);
      el.textContent = (end * (1 - Math.pow(1 - p, 3))).toFixed(dec) + suf;
      if (p < 1) requestAnimationFrame(step);
    }
    requestAnimationFrame(step);
  }

  function show(e) {
    e.classList.add('in');
    var c = e.querySelector('[data-count]');
    if (c && !reduce) count(c);
  }

  if (!('IntersectionObserver' in window) || reduce) {
    els.forEach(show);
    return;
  }
  var io = new IntersectionObserver(function (entries) {
    entries.forEach(function (en) {
      if (en.isIntersecting) { show(en.target); io.unobserve(en.target); }
    });
  }, { threshold: 0.12 });

  els.forEach(function (e, i) {
    e.classList.add('reveal');
    e.style.transitionDelay = (i % 4) * 70 + 'ms';
    io.observe(e);
  });
  // safety net: never leave content hidden
  setTimeout(function () { els.forEach(function (e) { e.classList.add('in'); }); }, 3500);
})();


(() => {
  const STORAGE_KEY = 'ros2-docker-course-progress-v2';
  const DISTRO_KEY = 'ros2-docker-course-distro-v2';
  const modules = [...document.querySelectorAll('.module')];
  const doneBoxes = [...document.querySelectorAll('.module-done')];
  const progressBar = document.getElementById('progressBar');
  const progressText = document.getElementById('progressText');
  const distroSelect = document.getElementById('distroSelect');
  const searchBox = document.getElementById('searchBox');

  function loadProgress() {
    let saved = {};
    try { saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}'); } catch (_) {}
    doneBoxes.forEach(box => { box.checked = Boolean(saved[box.dataset.module]); });
    updateProgress();
  }

  function saveProgress() {
    const saved = {};
    doneBoxes.forEach(box => { saved[box.dataset.module] = box.checked; });
    localStorage.setItem(STORAGE_KEY, JSON.stringify(saved));
    updateProgress();
  }

  function updateProgress() {
    const done = doneBoxes.filter(box => box.checked).length;
    const total = doneBoxes.length;
    const percent = total ? Math.round(done / total * 100) : 0;
    progressBar.style.width = `${percent}%`;
    progressText.textContent = `${done} / ${total} ステップ完了（${percent}%）`;
  }

  doneBoxes.forEach(box => box.addEventListener('change', saveProgress));

  function applyDistro(distro) {
    document.querySelectorAll('[data-distro]').forEach(el => { el.textContent = distro; });
    document.querySelectorAll('[data-distro-template]').forEach(el => {
      el.textContent = el.dataset.distroTemplate.replaceAll('{{DISTRO}}', distro);
    });
    localStorage.setItem(DISTRO_KEY, distro);
  }

  const savedDistro = localStorage.getItem(DISTRO_KEY) || 'lyrical';
  distroSelect.value = savedDistro;
  applyDistro(savedDistro);
  distroSelect.addEventListener('change', e => applyDistro(e.target.value));

  document.querySelectorAll('pre code').forEach(code => {
    const wrap = code.closest('.code-wrap');
    const title = wrap?.querySelector('.code-title');
    if (!title) return;
    const button = document.createElement('button');
    button.className = 'copy-btn';
    button.type = 'button';
    button.textContent = 'コピー';
    button.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(code.textContent);
        button.textContent = 'コピー済み';
        button.classList.add('copied');
        setTimeout(() => { button.textContent = 'コピー'; button.classList.remove('copied'); }, 1400);
      } catch (_) {
        button.textContent = '選択してコピー';
      }
    });
    title.appendChild(button);
  });

  document.querySelectorAll('.tabs').forEach(tabRoot => {
    const buttons = [...tabRoot.querySelectorAll('.tab-button')];
    const panels = [...tabRoot.querySelectorAll('.tab-panel')];
    buttons.forEach(button => button.addEventListener('click', () => {
      buttons.forEach(b => b.classList.toggle('active', b === button));
      panels.forEach(p => p.classList.toggle('active', p.id === button.dataset.target));
    }));
  });

  document.querySelectorAll('.quiz').forEach(quiz => {
    const check = quiz.querySelector('.quiz-check');
    const result = quiz.querySelector('.quiz-result');
    check?.addEventListener('click', () => {
      const selected = quiz.querySelector('input[type="radio"]:checked');
      if (!selected) {
        result.textContent = '選択肢を選んでください。';
        result.className = 'quiz-result ng';
        return;
      }
      const ok = selected.value === quiz.dataset.answer;
      result.textContent = ok ? `正解です。${quiz.dataset.explanation}` : `もう一度。${quiz.dataset.explanation}`;
      result.className = `quiz-result ${ok ? 'ok' : 'ng'}`;
    });
  });

  searchBox.addEventListener('input', () => {
    const q = searchBox.value.trim().toLowerCase();
    modules.forEach(module => {
      const text = module.textContent.toLowerCase();
      module.hidden = q && !text.includes(q);
    });
  });

  const navLinks = [...document.querySelectorAll('.nav-list a')];
  const observer = new IntersectionObserver(entries => {
    const visible = entries.filter(e => e.isIntersecting).sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
    if (!visible) return;
    navLinks.forEach(link => link.classList.toggle('active', link.getAttribute('href') === `#${visible.target.id}`));
  }, { rootMargin: '-15% 0px -70% 0px', threshold: [0.05, 0.2, 0.5] });
  modules.forEach(module => observer.observe(module));

  loadProgress();
})();

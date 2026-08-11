(() => {
  "use strict";

  const STORAGE_KEY = "ros2-slam-navigation-course-progress-v1";
  const THEME_KEY = "ros2-slam-navigation-course-theme-v1";

  const courseContent = document.getElementById("courseContent");
  const navList = document.getElementById("navList");
  const searchBox = document.getElementById("searchBox");
  const topicFilter = document.getElementById("topicFilter");
  const searchStatus = document.getElementById("searchStatus");
  const progressBar = document.getElementById("progressBar");
  const progressText = document.getElementById("progressText");
  const progressPercent = document.getElementById("progressPercent");
  const themeButton = document.getElementById("themeButton");
  const toTopButton = document.getElementById("toTopButton");

  const topicByIndex = [
    "foundation", "foundation", "foundation", "foundation", "foundation", "foundation",
    "integration", "integration",
    "practice",
    "handheld", "handheld", "handheld",
    "deployment", "deployment", "deployment",
    "reference", "reference", "reference", "reference"
  ];

  const chapterLabels = [
    "GOAL", "00", "01", "02", "03", "04", "05", "06", "07", "08",
    "09", "10", "11", "12", "13", "14", "15", "16", "17"
  ];

  function safeJsonParse(value, fallback) {
    try {
      return JSON.parse(value);
    } catch (_) {
      return fallback;
    }
  }

  function applyInitialTheme() {
    const saved = localStorage.getItem(THEME_KEY);
    const theme = saved || "dark";
    document.documentElement.dataset.theme = theme;
    themeButton.textContent = theme === "light" ? "☾" : "☼";
    themeButton.title = theme === "light" ? "ダーク表示へ切り替える" : "ライト表示へ切り替える";
  }

  applyInitialTheme();

  const modules = [...courseContent.querySelectorAll("section.level2")];

  modules.forEach((module, index) => {
    module.classList.add("module");
    module.dataset.topic = topicByIndex[index] || "reference";
    module.dataset.module = `chapter-${String(index).padStart(2, "0")}`;

    const heading = module.querySelector(":scope > h2");
    if (!heading) return;

    module.dataset.originalId = module.id || "";
    module.id = module.dataset.module;

    const header = document.createElement("div");
    header.className = "module-head";

    const titleGroup = document.createElement("div");
    const kicker = document.createElement("p");
    kicker.className = "module-kicker";
    kicker.textContent = index === 0 ? "LEARNING GOAL" : `CHAPTER ${chapterLabels[index]}`;

    const doneLabel = document.createElement("label");
    doneLabel.className = "done-label";
    doneLabel.innerHTML = `<input class="module-done" type="checkbox" data-module="${module.dataset.module}">完了`;

    titleGroup.append(kicker, heading);
    header.append(titleGroup, doneLabel);
    module.prepend(header);

    const firstParagraph = [...module.children].find((child) => child.tagName === "P");
    if (firstParagraph) firstParagraph.classList.add("module-lead");

    const navLink = document.createElement("a");
    navLink.href = `#${module.id}`;
    navLink.dataset.module = module.dataset.module;
    navLink.innerHTML = `<span class="nav-number">${chapterLabels[index] || String(index).padStart(2, "0")}</span><span>${heading.textContent}</span>`;
    navList.append(navLink);
  });

  document.querySelectorAll("#courseContent table").forEach((table) => {
    if (table.parentElement?.classList.contains("table-scroll")) return;
    const wrap = document.createElement("div");
    wrap.className = "table-scroll";
    table.before(wrap);
    wrap.append(table);
  });

  document.querySelectorAll("#courseContent p").forEach((paragraph) => {
    const text = paragraph.textContent.trim();
    if (text === "合格条件：" || text.startsWith("重要なのは、") || text.startsWith("重要なのは、")) {
      paragraph.classList.add("chapter-summary");
    }
  });

  function codeLanguage(pre, code) {
    const classes = [...pre.classList, ...code.classList].join(" ").toLowerCase();
    if (classes.includes("bash")) return "Bash / ROS 2 command";
    if (classes.includes("python")) return "Python";
    if (classes.includes("yaml")) return "YAML";
    if (classes.includes("json")) return "JSON";
    if (classes.includes("xml")) return "XML";
    if (classes.includes("text")) return "Frame / data flow";
    return "Command / configuration";
  }

  const mermaidBlocks = [];

  document.querySelectorAll("#courseContent pre").forEach((pre) => {
    const code = pre.querySelector("code");
    if (!code) return;

    const isMermaid = pre.classList.contains("mermaid") || code.classList.contains("mermaid");
    const source = code.textContent;
    const sourceContainer = pre.parentElement?.classList.contains("sourceCode") ? pre.parentElement : pre;

    if (isMermaid) {
      const shell = document.createElement("div");
      shell.className = "diagram-shell";
      const title = document.createElement("p");
      title.className = "diagram-title";
      title.textContent = "CONNECTION DIAGRAM";
      const diagram = document.createElement("pre");
      diagram.className = "mermaid";
      diagram.textContent = source;
      shell.append(title, diagram);
      sourceContainer.replaceWith(shell);
      mermaidBlocks.push(diagram);
      return;
    }

    let wrap;
    if (sourceContainer === pre) {
      wrap = document.createElement("div");
      wrap.className = "code-wrap";
      pre.before(wrap);
      wrap.append(pre);
    } else {
      wrap = sourceContainer;
      wrap.classList.add("code-wrap");
    }

    const title = document.createElement("div");
    title.className = "code-title";
    const titleText = document.createElement("span");
    titleText.textContent = codeLanguage(pre, code);
    const copyButton = document.createElement("button");
    copyButton.type = "button";
    copyButton.className = "copy-btn";
    copyButton.textContent = "コピー";
    copyButton.addEventListener("click", async () => {
      try {
        await navigator.clipboard.writeText(source);
        copyButton.textContent = "コピー済み";
        copyButton.classList.add("copied");
      } catch (_) {
        copyButton.textContent = "選択してコピー";
      }
      window.setTimeout(() => {
        copyButton.textContent = "コピー";
        copyButton.classList.remove("copied");
      }, 1500);
    });
    title.append(titleText, copyButton);
    wrap.prepend(title);
  });

  async function renderMermaid() {
    if (!mermaidBlocks.length) return;
    if (!window.mermaid) {
      mermaidBlocks.forEach((block) => block.classList.add("diagram-fallback"));
      return;
    }

    window.mermaid.initialize({
      startOnLoad: false,
      securityLevel: "loose",
      theme: "base",
      flowchart: {
        htmlLabels: true,
        curve: "basis",
        useMaxWidth: true
      },
      themeVariables: {
        background: "#070c17",
        primaryColor: "#17243d",
        primaryBorderColor: "#3b527b",
        primaryTextColor: "#edf3ff",
        secondaryColor: "#1d2c49",
        secondaryTextColor: "#edf3ff",
        tertiaryColor: "#121c31",
        tertiaryTextColor: "#edf3ff",
        lineColor: "#58d6c7",
        textColor: "#edf3ff",
        fontFamily: "Noto Sans JP Variable, -apple-system, BlinkMacSystemFont, Segoe UI, Noto Sans JP, sans-serif",
        fontSize: "15px"
      }
    });

    try {
      await window.mermaid.run({ nodes: mermaidBlocks });
    } catch (error) {
      console.warn("Mermaid rendering failed", error);
      mermaidBlocks.forEach((block) => block.classList.add("diagram-fallback"));
    }
  }

  renderMermaid();

  const doneBoxes = [...document.querySelectorAll(".module-done")];
  const navLinks = [...document.querySelectorAll("#navList a")];

  function savedProgress() {
    return safeJsonParse(localStorage.getItem(STORAGE_KEY) || "{}", {});
  }

  function updateProgress() {
    const completed = doneBoxes.filter((box) => box.checked).length;
    const total = doneBoxes.length;
    const percent = total ? Math.round((completed / total) * 100) : 0;
    progressBar.style.width = `${percent}%`;
    progressText.textContent = `${completed} / ${total} 章完了`;
    progressPercent.textContent = `${percent}%`;

    navLinks.forEach((link) => {
      const box = doneBoxes.find((candidate) => candidate.dataset.module === link.dataset.module);
      link.classList.toggle("done", Boolean(box?.checked));
    });
  }

  function loadProgress() {
    const saved = savedProgress();
    doneBoxes.forEach((box) => {
      box.checked = Boolean(saved[box.dataset.module]);
    });
    updateProgress();
  }

  function saveProgress() {
    const saved = {};
    doneBoxes.forEach((box) => {
      saved[box.dataset.module] = box.checked;
    });
    localStorage.setItem(STORAGE_KEY, JSON.stringify(saved));
    updateProgress();
  }

  doneBoxes.forEach((box) => box.addEventListener("change", saveProgress));
  loadProgress();

  function applyFilters() {
    const query = searchBox.value.trim().toLocaleLowerCase("ja");
    const topic = topicFilter.value;
    let visible = 0;

    modules.forEach((module) => {
      const matchesQuery = !query || module.textContent.toLocaleLowerCase("ja").includes(query);
      const matchesTopic = topic === "all" || module.dataset.topic === topic;
      const show = matchesQuery && matchesTopic;
      module.hidden = !show;
      if (show) visible += 1;
    });

    navLinks.forEach((link) => {
      const target = document.querySelector(link.getAttribute("href"));
      link.hidden = Boolean(target?.hidden);
    });

    if (query || topic !== "all") {
      searchStatus.textContent = `${visible} / ${modules.length} 章を表示`;
    } else {
      searchStatus.textContent = "";
    }
  }

  searchBox.addEventListener("input", applyFilters);
  topicFilter.addEventListener("change", applyFilters);

  const observer = new IntersectionObserver((entries) => {
    const visible = entries
      .filter((entry) => entry.isIntersecting && !entry.target.hidden)
      .sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
    if (!visible) return;
    navLinks.forEach((link) => {
      link.classList.toggle("active", link.getAttribute("href") === `#${visible.target.id}`);
    });
  }, {
    rootMargin: "-15% 0px -70% 0px",
    threshold: [0.05, 0.18, 0.45]
  });

  modules.forEach((module) => observer.observe(module));

  document.querySelectorAll('a[href^="http"]').forEach((link) => {
    link.target = "_blank";
    link.rel = "noreferrer noopener";
  });

  document.querySelectorAll(".quiz").forEach((quiz) => {
    const button = quiz.querySelector(".quiz-check");
    const result = quiz.querySelector(".quiz-result");
    button?.addEventListener("click", () => {
      const selected = quiz.querySelector('input[type="radio"]:checked');
      if (!selected) {
        result.textContent = "選択肢を選んでください。";
        result.className = "quiz-result ng";
        return;
      }
      const correct = selected.value === quiz.dataset.answer;
      result.textContent = `${correct ? "正解です。" : "もう一度確認しましょう。"}${quiz.dataset.explanation}`;
      result.className = `quiz-result ${correct ? "ok" : "ng"}`;
    });
  });

  themeButton.addEventListener("click", () => {
    const next = document.documentElement.dataset.theme === "light" ? "dark" : "light";
    document.documentElement.dataset.theme = next;
    localStorage.setItem(THEME_KEY, next);
    themeButton.textContent = next === "light" ? "☾" : "☼";
    themeButton.title = next === "light" ? "ダーク表示へ切り替える" : "ライト表示へ切り替える";
  });

  document.getElementById("printButton")?.addEventListener("click", () => window.print());

  document.getElementById("resetProgress")?.addEventListener("click", () => {
    const shouldReset = window.confirm("この教材の完了チェックをすべて解除しますか？");
    if (!shouldReset) return;
    localStorage.removeItem(STORAGE_KEY);
    doneBoxes.forEach((box) => {
      box.checked = false;
    });
    updateProgress();
  });

  window.addEventListener("scroll", () => {
    toTopButton.classList.toggle("visible", window.scrollY > 800);
  }, { passive: true });

  toTopButton.addEventListener("click", () => {
    window.scrollTo({ top: 0, behavior: "smooth" });
  });
})();

document.addEventListener("DOMContentLoaded", function () {
    const button = document.createElement("button");
    button.innerText = "🌙/☀️";
    button.style.position = "fixed";
    button.style.bottom = "20px";
    button.style.right = "20px";
    button.style.padding = "10px";
    button.style.borderRadius = "8px";
    button.style.border = "none";
    button.style.cursor = "pointer";
    button.style.zIndex = "9999";
    button.style.background = "#036B9D";
    button.style.color = "white";
    document.body.appendChild(button);

    const mainImage = document.querySelector(".main-logo");

    // Verifica se já tinha preferência salva
    if (localStorage.getItem("dark-mode") === "enabled") {
        document.body.classList.add("dark-mode");
        if (mainImage) mainImage.src = "_static/images/logo-dark.svg";
    }

    button.addEventListener("click", function () {
        document.body.classList.toggle("dark-mode");

        if (document.body.classList.contains("dark-mode")) {
            localStorage.setItem("dark-mode", "enabled");
            if (mainImage) mainImage.src = "_static/images/logo-dark.svg";
        } else {
            localStorage.setItem("dark-mode", "disabled");
            if (mainImage) mainImage.src = "_static/images/logo-light.svg";
        }
    });
});

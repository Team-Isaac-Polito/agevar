# AGEVAR - Workflow per Sincronizzazione Branch

## 📋 Procedura Standard per Aggiornamenti

### 🔄 **Quando modifichi il MAIN e vuoi portare le modifiche al DEMO-LOOP:**

```bash
# 1. Assicurati che main sia committato
git checkout main
git add .
git commit -m "Descrizione modifiche"

# 2. Vai al demo-loop
git checkout demo-loop

# 3. Sincronizza le modifiche (metodo semplice)
git checkout main -- agevar.m README.md
# OPPURE per specifici file:
# git checkout main -- README.md  (solo README)

# 4. Verifica che il loop funzioni ancora
# Controlla che le modifiche non rompano la struttura while(1)

# 5. Committa le modifiche sincronizzate
git add .
git commit -m "Sync with main: descrizione"
```

### 🔄 **Quando modifichi il DEMO-LOOP e vuoi portare le modifiche al MAIN:**

```bash
# 1. Assicurati che demo-loop sia committato
git checkout demo-loop
git add .
git commit -m "Descrizione modifiche"

# 2. Vai al main
git checkout main

# 3. Applica selettivamente le modifiche
# (copia manualmente le parti che servono, evitando il while(1))

# 4. Committa
git add .
git commit -m "Apply demo-loop improvements: descrizione"
```

## 🎛️ **File Condivisi vs Specifici**

### **Sempre Sincronizzati:**

- `README.md` - Documentazione unificata
- Funzioni utility (`Rotz`, `plotModule*`)
- Parametri robot (geometria, etc.)

### **Specifici per Branch:**

- **main**: Struttura semplice, parametri configurabili in alto
- **demo-loop**: Struttura `while(1)` con loop annidati

## ⚡ **Comandi Rapidi**

### Sync README only:

```bash
git checkout demo-loop
git checkout main -- README.md
git commit -m "Sync README with main"
```

### Sync tutto tranne la struttura loop:

```bash
git checkout demo-loop
git checkout main -- agevar.m
# Poi editare manualmente per ripristinare while(1)
git commit -m "Sync core functionality with main"
```

## 🚨 **Cosa Evitare**

- Non usare `git merge` tra questi branch (crea conflitti complessi)
- Non perdere la struttura `while(1)` del demo-loop
- Sempre testare dopo la sincronizzazione

## ✅ **Workflow Testato**

Questa procedura è stata testata il 25/07/2025 e funziona correttamente.
